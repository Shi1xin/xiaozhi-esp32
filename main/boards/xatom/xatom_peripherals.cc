#include "xatom_peripherals.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

static const char* TAG = "XAtomPeripherals";

namespace {
constexpr int kPollMs = 20;
constexpr int64_t kPmicIntervalUs = 1000 * 1000;  // monotonic 1000ms deadline
constexpr int kRadarChunk = 128;
constexpr int kMaxVersionAttempts = 3;
constexpr int64_t kVersionGapUs = 1000 * 1000;
constexpr int64_t kRadarWakeGapUs = 1000 * 1000;
}  // namespace

bool XAtomPeripherals::RegRead(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out, size_t len) {
    return dev != nullptr && i2c_master_transmit_receive(dev, &reg, 1, out, len, 100) == ESP_OK;
}

// Same as RegRead but reports the real SDK result for last_error tracking.
static esp_err_t RegReadRc(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out, size_t len) {
    if (dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(dev, &reg, 1, out, len, 100);
}

esp_err_t XAtomPeripherals::Start() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = XATOM_I2C_PORT,
        .sda_io_num = XATOM_I2C_SDA_PIN,
        .scl_io_num = XATOM_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {.enable_internal_pullup = true},
    };
    esp_err_t rc = i2c_new_master_bus(&bus_cfg, &i2c_bus_);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed rc=%s", esp_err_to_name(rc));
        pmic_.last_error = rc;
        imu_.last_error = rc;
        return rc;
    }

    InitPmic(&pmic_dev_);
    InitImu(&imu_dev_);

    // Radar UART2: RX buffer 2048, no flow control.
    const esp_err_t install_rc = uart_driver_install(XATOM_RADAR_UART, 2048, 0, 0, NULL, 0);
    if (install_rc == ESP_OK) {
        uart_config_t ucfg = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        rc = uart_param_config(XATOM_RADAR_UART, &ucfg);
        if (rc == ESP_OK) {
            rc = uart_set_pin(XATOM_RADAR_UART, XATOM_RADAR_TX_PIN, XATOM_RADAR_RX_PIN,
                              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        }
        radar_.available = rc == ESP_OK;
        radar_.last_error = rc;
    } else {
        radar_.last_error = install_rc;  // actual SDK install error
        ESP_LOGE(TAG, "radar uart driver install failed rc=%s", esp_err_to_name(install_rc));
    }

    SendRadarVersionRequest();
    next_version_us_ = esp_timer_get_time() + kVersionGapUs;
    next_pmic_us_ = esp_timer_get_time();  // first PMIC sample immediately

    stopped_ = false;
    running_ = true;
    if (xTaskCreate([](void* p) { static_cast<XAtomPeripherals*>(p)->WorkerLoop(); },
                    "xatom_periph", 4096, this, 5, &worker_) != pdPASS) {
        ESP_LOGE(TAG, "peripheral worker create failed");
        running_ = false;
        stopped_ = true;
        pmic_.valid = false;
        pmic_.last_error = ESP_ERR_NO_MEM;
        imu_.valid = false;
        imu_.last_error = ESP_ERR_NO_MEM;
        radar_.valid = false;
        radar_.last_error = ESP_ERR_NO_MEM;
        worker_ = nullptr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void XAtomPeripherals::Stop() {
    // Cooperative shutdown: the worker owns its lifecycle and deletes itself
    // after leaving the loop, so it can never be killed while holding the
    // snapshot mutex. Join unbounded — in-flight I2C transactions may take
    // 100ms each; abandoning a live worker would let it touch freed members.
    running_ = false;
    while (!stopped_.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    worker_ = nullptr;
}

void XAtomPeripherals::InitPmic(i2c_master_dev_handle_t* dev) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XATOM_PMIC_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t rc = i2c_master_bus_add_device(i2c_bus_, &cfg, dev);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "pmic add failed rc=%s", esp_err_to_name(rc));
        pmic_.last_error = rc;
        return;
    }
    uint8_t chip = 0;
    rc = RegReadRc(*dev, 0x03, &chip, 1);
    if (rc != ESP_OK || chip != 0x4A) {
        ESP_LOGE(TAG, "pmic chip id mismatch raw=0x%02x", chip);
        pmic_.last_error = rc != ESP_OK ? rc : ESP_ERR_INVALID_RESPONSE;
        i2c_master_bus_rm_device(*dev);
        *dev = nullptr;
        return;
    }
    pmic_.available = true;
    // Enable die-temperature ADC only when missing: read 0x30, write 0x30|0x10.
    uint8_t reg30 = 0;
    if (RegRead(*dev, 0x30, &reg30, 1) && (reg30 & 0x10) == 0) {
        reg30 |= 0x10;
        uint8_t buf[2] = {0x30, reg30};
        rc = i2c_master_transmit(*dev, buf, 2, 100);
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "pmic temp-adc enable write failed");
            pmic_.last_error = rc;
        }
    }
}

void XAtomPeripherals::InitImu(i2c_master_dev_handle_t* dev) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XATOM_IMU_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t rc = i2c_master_bus_add_device(i2c_bus_, &cfg, dev);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "imu add failed rc=%s", esp_err_to_name(rc));
        imu_.last_error = rc;
        return;
    }
    uint8_t who = 0;
    rc = RegReadRc(*dev, 0x00, &who, 1);
    if (rc != ESP_OK || who != 0x05) {
        ESP_LOGE(TAG, "imu who_am_i mismatch raw=0x%02x", who);
        imu_.last_error = rc != ESP_OK ? rc : ESP_ERR_INVALID_RESPONSE;
        i2c_master_bus_rm_device(*dev);
        *dev = nullptr;
        return;
    }
    // CTRL1=0x60, CTRL2=0x25 (+/-8g 250Hz), CTRL3=0x66 (stock gyro preset),
    // CTRL7=0x03 (accel+gyro enabled).
    const uint8_t writes[][2] = {{0x02, 0x60}, {0x03, 0x25}, {0x04, 0x66}, {0x08, 0x03}};
    for (auto& w : writes) {
        rc = i2c_master_transmit(*dev, w, 2, 100);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "imu cfg reg=0x%02x write failed", w[0]);
            imu_.last_error = rc;
            i2c_master_bus_rm_device(*dev);
            *dev = nullptr;
            return;
        }
    }
    imu_.available = true;
}

void XAtomPeripherals::WorkerLoop() {
    while (running_) {
        const int64_t now = esp_timer_get_time();
        if (now >= next_pmic_us_) {
            next_pmic_us_ = now + kPmicIntervalUs;
            SamplePmic();
        }
        SampleImu();
        DrainRadar();
        if (radar_.available && version_attempts_ < kMaxVersionAttempts &&
            !radar_.version_received && esp_timer_get_time() >= next_version_us_) {
            SendRadarVersionRequest();
            next_version_us_ = esp_timer_get_time() + kVersionGapUs;
        }
        // Raw GPIO reads stay outside the snapshot mutex.
        const int btn = gpio_get_level(XATOM_BUTTON_PIN);
        const int rio = gpio_get_level(XATOM_RADAR_IO_PIN);
        const int irq = gpio_get_level(XATOM_IMU_IRQ_PIN);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            inputs_.button = btn;
            inputs_.radar_io = rio;
            inputs_.imu_irq = irq;
        }
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
    stopped_.store(true, std::memory_order_release);
    vTaskDelete(NULL);
}

void XAtomPeripherals::SamplePmic() {
    if (!pmic_.available)
        return;
    // Adjacent register pairs in one transaction each; A4 read alone. The
    // first failing transaction supplies the real SDK error.
    uint8_t st[2], bat[2], vbus[2], temp[2], soc = 0;
    esp_err_t rc = RegReadRc(pmic_dev_, 0x00, st, 2);
    if (rc == ESP_OK)
        rc = RegReadRc(pmic_dev_, 0x34, bat, 2);
    if (rc == ESP_OK)
        rc = RegReadRc(pmic_dev_, 0x38, vbus, 2);
    if (rc == ESP_OK)
        rc = RegReadRc(pmic_dev_, 0x3C, temp, 2);
    if (rc == ESP_OK)
        rc = RegReadRc(pmic_dev_, 0xA4, &soc, 1);
    xatom::PmicSnapshot next = pmic_;
    next.sample_us = esp_timer_get_time();
    if (rc == ESP_OK && xatom::PmicDecode(st[0], st[1], bat[0], bat[1], vbus[0], vbus[1], temp[0],
                                          temp[1], soc, &next)) {
        next.valid = true;
        next.last_error = 0;
    } else {
        next.valid = false;
        next.last_error = rc != ESP_OK ? static_cast<int>(rc) : ESP_ERR_INVALID_RESPONSE;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pmic_ = next;
}

void XAtomPeripherals::SampleImu() {
    if (!imu_.available)
        return;
    uint8_t raw[12];
    const esp_err_t rc = RegReadRc(imu_dev_, 0x35, raw, sizeof(raw));
    const int irq = gpio_get_level(XATOM_IMU_IRQ_PIN);
    xatom::ImuSample next = imu_;
    next.sample_us = esp_timer_get_time();
    next.irq_level = irq;
    if (rc == ESP_OK && xatom::ImuDecode(raw, sizeof(raw), &next)) {
        next.valid = true;
        next.last_error = 0;
    } else {
        next.valid = false;
        next.last_error = rc != ESP_OK ? static_cast<int>(rc) : ESP_ERR_INVALID_RESPONSE;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    imu_ = next;
}

void XAtomPeripherals::SendRadarVersionRequest() {
    if (!radar_.available)
        return;  // send only when UART2 is initialized
    size_t len = 0;
    const uint8_t* req = xatom::RadarVersionRequest(&len);
    const int written = uart_write_bytes(XATOM_RADAR_UART, req, len);
    version_attempts_++;  // count every attempt, including failed writes
    ESP_LOGI(TAG, "radar version request attempt=%d written=%d", version_attempts_, written);
    if (written != (int)len) {
        const int rc = written < 0 ? written : ESP_ERR_INVALID_SIZE;
        std::lock_guard<std::mutex> lock(mutex_);
        radar_.last_error = rc;
    }
}

void XAtomPeripherals::DrainRadar() {
    if (!radar_.available)
        return;
    uint8_t buf[kRadarChunk];
    // Nonblocking bounded read; each byte is appended exactly once and then
    // complete frames are consumed until none remain.
    const int n = uart_read_bytes(XATOM_RADAR_UART, buf, sizeof(buf), 0);
    if (n <= 0)
        return;
    xatom::XAtomRadarParser::Frame frame;
    for (int i = 0; i < n; ++i) {
        parser_.Append(buf[i]);
        while (parser_.Next(&frame)) {
            HandleRadarFrame(frame);
        }
    }
}

void XAtomPeripherals::HandleRadarFrame(const xatom::XAtomRadarParser::Frame& frame) {
    char ver[48] = {};
    xatom::RadarTelemetry tel;
    if (xatom::RadarParseVersion(frame, ver, sizeof(ver))) {
        ESP_LOGI(TAG, "radar version=%s", ver);
        std::lock_guard<std::mutex> lock(mutex_);
        strncpy(radar_.version, ver, sizeof(radar_.version) - 1);
        radar_.version[sizeof(radar_.version) - 1] = '\0';
        radar_.version_received = true;  // telemetry validity stays separate
        return;
    }
    if (xatom::RadarParseTelemetry(frame, &tel)) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            radar_.state = tel.state;
            radar_.distance_raw = tel.distance_raw;
            radar_.signal_raw = tel.signal_raw;
            radar_.sample_us = esp_timer_get_time();
            radar_.valid = true;
        }
        // Bounded wake request: at most one pending, at most every 1000ms.
        if (tel.state != 0 && !radar_wake_pending_.load(std::memory_order_acquire)) {
            const int64_t now = esp_timer_get_time();
            if (now - last_radar_wake_us_ >= kRadarWakeGapUs) {
                last_radar_wake_us_ = now;
                radar_wake_pending_.store(true, std::memory_order_release);
                if (wake_callback_) {
                    wake_callback_();
                }
            }
        }
    }
}

void XAtomPeripherals::PmicPowerOff() {
    if (!pmic_.available)
        return;
    uint8_t reg = 0;
    if (!RegRead(pmic_dev_, 0x10, &reg, 1)) {
        ESP_LOGE(TAG, "pmic poweroff read failed");
        return;
    }
    const uint8_t buf[2] = {0x10, static_cast<uint8_t>(reg | 0x01)};
    ESP_LOGI(TAG, "pmic poweroff write rc=%s",
             esp_err_to_name(i2c_master_transmit(pmic_dev_, buf, 2, 100)));
}

xatom::PmicSnapshot XAtomPeripherals::PowerSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pmic_;
}

xatom::ImuSample XAtomPeripherals::MotionSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_;
}

XAtomRadarSnapshot XAtomPeripherals::RadarSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return radar_;
}

XAtomInputSnapshot XAtomPeripherals::InputSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return inputs_;
}
