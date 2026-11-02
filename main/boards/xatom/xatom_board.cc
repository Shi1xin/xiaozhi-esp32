/*
 * X-Atom board — Linx intercom hardware on ESP32-S3.
 *
 * Dual-network (Wi-Fi default + ML307 4G), ES8311 stereo reference codec,
 * AXP2101 PMIC, QMI8658 IMU, RKB1161 radar, 4-pixel WS2812 strip, one
 * active-low button, GPIO39 startup control (stock HIGH, role unverified).
 */

#include "application.h"
#include "audio_codec.h"
#include "board.h"
#include "button.h"
#include "codecs/dummy_audio_codec.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "display.h"
#include "dual_network_board.h"
#include "led/circular_strip.h"
#include "led/led.h"
#include "mcp_server.h"
#include "power_save_timer.h"
#include "settings.h"
#include "ssid_manager.h"
#include "system_reset.h"
#include "wifi_board.h"
#include "xatom_peripherals.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <at_modem.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <memory>
#include <mutex>

static const char* TAG = "XAtomBoard";

namespace {
// Deferred one-shot system actions; only one may be pending at a time.
enum PendingAction : int {
    kActionNone = 0,
    kActionShutdown,
    kActionSwitchToWifi,
    kActionSwitchTo4g,
    kActionRescueWifi,
    kActionWifiConfig,
    kActionReboot,
};

// Board-local NVS flag: boot once into forced Wi-Fi provisioning after a
// rescue request from modem mode. Documented in README; not a protocol key.
constexpr const char* kProvisionKey = "provision";

void AddCommonFields(cJSON* obj, bool available, bool valid, int last_error, int64_t sample_us) {
    cJSON_AddBoolToObject(obj, "available", available);
    cJSON_AddBoolToObject(obj, "valid", valid);
    cJSON_AddNumberToObject(obj, "last_error", last_error);
    cJSON_AddNumberToObject(obj, "sample_us", (double)sample_us);
}

void AddNumberOrNull(cJSON* obj, const char* key, bool valid, double value) {
    if (valid) {
        cJSON_AddNumberToObject(obj, key, value);
    } else {
        cJSON_AddNullToObject(obj, key);
    }
}
}  // namespace

class XAtomBoard : public DualNetworkBoard {
private:
    XAtomPeripherals peripherals_;
    AudioCodec* codec_ = nullptr;
    CircularStrip* led_ = nullptr;
    Button button_;
    PowerSaveTimer* power_save_timer_ = nullptr;

    esp_timer_handle_t action_timer_ = nullptr;
    std::atomic<int> pending_action_{kActionNone};

    // PowerSaveTimer fields are owned by ESP_TIMER_TASK: activity requests
    // coalesce into this board-owned 1000us one-shot timer on the same task;
    // only its callback may call WakeUp/SetEnabled.
    esp_timer_handle_t activity_timer_ = nullptr;
    std::atomic<bool> activity_pending_{false};

    std::atomic<bool> modem_ready_{false};
    std::atomic<int> remembered_level_{static_cast<int>(PowerSaveLevel::PERFORMANCE)};
    std::atomic<uint32_t> connection_generation_{0};
    // Worker-owned apply state: written only inside the control worker, so a
    // network callback can never erase an in-flight apply's record.
    int last_applied_level_ = -1;
    uint32_t last_applied_generation_ = UINT32_MAX;
    NetworkEventCallback forward_callback_;

    QueueHandle_t control_queue_ = nullptr;
    TaskHandle_t control_worker_ = nullptr;

    void InitializeButton() {
        button_.OnClick([this]() {
            RequestActivity();
            Application::GetInstance().Schedule(
                []() { Application::GetInstance().ToggleChatState(); });
        });
        button_.OnDoubleClick([this]() {
            RequestActivity();
            QueueAction(kActionReboot);
        });
        button_.OnMultipleClick(
            [this]() {
                RequestActivity();
                QueueAction(GetNetworkType() == NetworkType::WIFI ? kActionSwitchTo4g
                                                                  : kActionSwitchToWifi);
            },
            3);
        button_.OnLongPress([this]() {
            RequestActivity();
            if (GetNetworkType() == NetworkType::ML307) {
                // Rescue: forced Wi-Fi provisioning boot.
                QueueAction(kActionRescueWifi);
            } else {
                // BluFi provisioning on the live Wi-Fi backend, keeping
                // existing credentials.
                QueueAction(kActionWifiConfig);
            }
        });
        // Distinct hold threshold for power-off; fires while still held.
        button_.OnLongPress(
            [this]() {
                RequestActivity();
                QueueAction(kActionShutdown);
            },
            7000);
    }

    // Coalesce activity requests onto the timer task; only the timer
    // callback touches PowerSaveTimer state. A rejected start releases both
    // the pending flag and the radar wake latch so neither stays set.
    void RequestActivity() {
        if (activity_timer_ == nullptr) {
            peripherals_.WakeAcknowledged();
            return;
        }
        if (activity_pending_.exchange(true, std::memory_order_acq_rel)) {
            return;  // a request is already coalesced
        }
        if (esp_timer_start_once(activity_timer_, 1000) != ESP_OK) {
            activity_pending_.store(false, std::memory_order_release);
            peripherals_.WakeAcknowledged();
        }
    }

    static void ActivityTimerCallback(void* arg) {
        auto* self = static_cast<XAtomBoard*>(arg);
        if (self->activity_pending_.exchange(false, std::memory_order_acq_rel)) {
            if (self->power_save_timer_ != nullptr) {
                self->power_save_timer_->WakeUp();
            }
            self->peripherals_.WakeAcknowledged();  // always release the latch
        }
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.xatom.get_state", "Get X-Atom power/motion/radar/input snapshot",
                    PropertyList(),
                    [this](const PropertyList&) -> ReturnValue { return BuildStateJson(); });
        mcp.AddUserOnlyTool(
            "self.xatom.shutdown", "Power off the device", PropertyList(),
            [this](const PropertyList&) -> ReturnValue { return QueueAction(kActionShutdown); });
        mcp.AddUserOnlyTool("self.xatom.switch_network", "Switch network (network: wifi | 4g)",
                            PropertyList({Property("network", kPropertyTypeString)}),
                            [this](const PropertyList& properties) -> ToolResult {
                                const std::string net = properties["network"].value<std::string>();
                                if (net != "wifi" && net != "4g") {
                                    return std::unexpected(std::string("invalid network: ") + net);
                                }
                                const bool want_wifi = net == "wifi";
                                const bool is_wifi = GetNetworkType() == NetworkType::WIFI;
                                if (want_wifi == is_wifi) {
                                    return ReturnValue{true};  // same network, no reboot
                                }
                                return ReturnValue{QueueAction(want_wifi ? kActionSwitchToWifi
                                                                         : kActionSwitchTo4g)};
                            });
    }

    // Queue a deferred system action; false when one is already pending.
    // The latch stays set until the scheduled application callback runs.
    bool QueueAction(int action) {
        if (action_timer_ == nullptr) {
            return false;
        }
        int expected = kActionNone;
        if (!pending_action_.compare_exchange_strong(expected, action)) {
            return false;
        }
        if (esp_timer_start_once(action_timer_, 1000 * 1000) != ESP_OK) {
            pending_action_.store(kActionNone);  // release latch on failure
            return false;
        }
        return true;
    }

    static void ActionTimerCallback(void* arg) {
        // Read without clearing: the app callback owns the latch so a second
        // request is still rejected while the first is queued.
        const int action = static_cast<XAtomBoard*>(arg)->pending_action_.load();
        if (action == kActionNone)
            return;
        Application::GetInstance().Schedule([action]() {
            auto* self = static_cast<XAtomBoard*>(&Board::GetInstance());
            self->pending_action_.store(kActionNone);
            switch (action) {
                case kActionShutdown:
                    self->DoShutdown();
                    break;
                case kActionReboot:
                    Application::GetInstance().Reboot();
                    break;
                case kActionSwitchToWifi:
                case kActionSwitchTo4g: {
                    // Settings commits in its destructor; leave the inner
                    // scope (flush NVS) before rebooting.
                    {
                        Settings net("network", true);
                        net.SetInt("type", action == kActionSwitchToWifi ? 0 : 1);
                    }
                    Application::GetInstance().Reboot();
                    break;
                }
                case kActionWifiConfig: {
                    auto* wifi = static_cast<WifiBoard*>(&self->GetCurrentBoard());
                    if (wifi != nullptr) {
                        wifi->EnterWifiConfigMode();
                    }
                    break;
                }
                case kActionRescueWifi: {
                    {
                        Settings net("network", true);
                        net.SetInt("type", 0);
                        Settings flag("xatom", true);
                        flag.SetBool(kProvisionKey, true);
                    }
                    Application::GetInstance().Reboot();
                    break;
                }
            }
        });
    }

    void DoShutdown() {
        ESP_LOGI(TAG, "shutdown: PA off, LEDs clear, gpio39 low, PMIC power off");
        if (codec_ != nullptr) {
            codec_->EnableOutput(false);  // PA follows output_enabled -> low
        }
        if (led_ != nullptr) {
            led_->SetAllColor({0, 0, 0});
        }
        gpio_set_level(XATOM_CONTROL39_PIN, 0);
        peripherals_.PmicPowerOff();  // reg 0x10 RMW |= 0x01, stock sequence
    }

    // Length-1 overwrite queue: SetPowerSaveLevel only enqueues.
    void ControlWorkerLoop() {
        int token = 0;
        while (xQueueReceive(control_queue_, &token, portMAX_DELAY) == pdTRUE) {
            // Only a wake token is queued; the desired level and connection
            // generation are read atomically at dequeue, so a stale queue
            // entry can never overwrite a newer request.
            const PowerSaveLevel level = static_cast<PowerSaveLevel>(remembered_level_.load());
            ApplyPowerSaveLevel(level, connection_generation_.load());
        }
    }

    void ApplyPowerSaveLevel(PowerSaveLevel level, uint32_t generation) {
        // Dedup identical applies only when level AND generation match the
        // last successful apply — a new Connected event during a blocking AT
        // forces the queued next apply, and a failed apply records nothing.
        if (last_applied_level_ == static_cast<int>(level) &&
            last_applied_generation_ == generation) {
            return;
        }
        if (GetNetworkType() == NetworkType::WIFI) {
            DualNetworkBoard::SetPowerSaveLevel(level);
            last_applied_level_ = static_cast<int>(level);
            last_applied_generation_ = generation;
            return;
        }
        if (!modem_ready_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "modem not ready; deferring power level %d", level);
            return;
        }
        // Verified SDK call: LOW_POWER -> AT+MLPMCFG="sleepmode",2,0;
        // other levels -> mode 0,0. SetSleepMode is virtual on AtModem.
        auto* modem = static_cast<AtModem*>(GetNetwork());
        const bool rc = modem->SetSleepMode(level == PowerSaveLevel::LOW_POWER, 0);
        if (rc) {
            last_applied_level_ = static_cast<int>(level);
            last_applied_generation_ = generation;
        }
        ESP_LOGI(TAG, "modem sleep mode rc=%d level=%d", rc, level);
    }

public:
    XAtomBoard()
        : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, ML307_DTR_PIN,
                           /*default_net_type=*/0),
          button_(XATOM_BUTTON_PIN, false, 3000) {
        // PA low before any output enable, per the documented pin contract.
        gpio_set_level(AUDIO_I2S_PA_GPIO, 0);
        gpio_set_direction(AUDIO_I2S_PA_GPIO, GPIO_MODE_OUTPUT);

        // Reproduce verified stock startup: GPIO39 HIGH (checked, logged).
        esp_err_t rc = gpio_set_level(XATOM_CONTROL39_PIN, 1);
        if (rc == ESP_OK) {
            rc = gpio_set_direction(XATOM_CONTROL39_PIN, GPIO_MODE_INPUT_OUTPUT);
        }
        ESP_LOGI(TAG, "gpio39 held high (stock startup level) rc=%s", esp_err_to_name(rc));

        // GPIO5 pullup + input-only pins for raw reads.
        gpio_config_t inputs = {
            .pin_bit_mask = (1ULL << XATOM_BUTTON_PIN) | (1ULL << XATOM_IMU_IRQ_PIN) |
                            (1ULL << XATOM_RADAR_IO_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&inputs);
        gpio_set_pull_mode(XATOM_BUTTON_PIN, GPIO_PULLUP_ONLY);

        // Timers, callbacks, and the wake callback must exist before the
        // peripheral worker can fire them.
        const esp_timer_create_args_t timer_args = {
            .callback = ActionTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "xatom_action",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &action_timer_));

        const esp_timer_create_args_t activity_args = {
            .callback = ActivityTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "xatom_activity",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&activity_args, &activity_timer_));
        peripherals_.SetWakeCallback([this]() { RequestActivity(); });

        // All timers must exist before the peripheral worker: a first radar
        // wake could otherwise schedule activity before the power timer
        // pointer is written, stranding the latch.
        power_save_timer_ = new PowerSaveTimer(-1, 60, -1);
        power_save_timer_->OnEnterSleepMode(
            []() { Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::LOW_POWER); });
        power_save_timer_->OnExitSleepMode(
            []() { Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE); });
        power_save_timer_->SetEnabled(true);

        if (peripherals_.Start() != ESP_OK) {
            ESP_LOGE(TAG, "peripheral worker failed to start; sensors stay invalid");
        }

        // Codec after bus init; probe ES8311 at 0x18 before constructing the
        // real codec, else fall back to the dummy and log.
        bool codec_found = false;
        if (peripherals_.I2cBus() != nullptr) {
            codec_found =
                i2c_master_probe(peripherals_.I2cBus(), XATOM_ES8311_ADDR >> 1, 100) == ESP_OK;
        }
        if (codec_found) {
            codec_ = new Es8311AudioCodec(
                peripherals_.I2cBus(), XATOM_I2C_PORT, AUDIO_INPUT_SAMPLE_RATE,
                AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK,
                AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_I2S_PA_GPIO,
                XATOM_ES8311_ADDR, /*use_mclk=*/false, /*pa_inverted=*/false,
                /*input_reference=*/true);
        } else {
            ESP_LOGE(TAG, "ES8311 not detected; audio unavailable");
            codec_ = new DummyAudioCodec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE);
        }

        led_ = new CircularStrip(XATOM_LED_PIN, XATOM_LED_COUNT);
        led_->SetBrightness(4, 2);

        control_queue_ = xQueueCreate(1, sizeof(int));
        if (control_queue_ != nullptr &&
            xTaskCreate([](void* p) { static_cast<XAtomBoard*>(p)->ControlWorkerLoop(); },
                        "xatom_ctrl", 3072, this, 4, &control_worker_) != pdPASS) {
            ESP_LOGE(TAG, "control worker create failed");
            control_worker_ = nullptr;
        }

        InitializeButton();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override { return codec_; }
    virtual Led* GetLed() override { return led_; }

    // --- power inputs -------------------------------------------------
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        const auto s = peripherals_.PowerSnapshot();
        if (!xatom::PmicBatteryLevelValid(s)) {
            return false;
        }
        level = s.battery_percent;
        charging = s.charging;
        discharging = s.discharging;
        return true;
    }

    virtual bool GetTemperature(float& celsius) override {
        const auto s = peripherals_.PowerSnapshot();
        if (!s.valid || !s.temperature_valid) {
            return false;  // leaves output untouched on failure
        }
        celsius = s.temperature_c;
        return true;
    }

    // --- network -------------------------------------------------------
    virtual void StartNetwork() override {
        // Rescue boot: forced provisioning flag set by the long-press path.
        Settings flag("xatom", true);
        const bool rescue = flag.GetBool(kProvisionKey, false);
        bool had_ssids = false;
        if (rescue && GetNetworkType() == NetworkType::WIFI) {
            had_ssids = !SsidManager::GetInstance().GetSsidList().empty();
        }
        DualNetworkBoard::StartNetwork();
        if (rescue && GetNetworkType() == NetworkType::WIFI) {
            flag.EraseKey(kProvisionKey);  // consume after this boot path
            if (had_ssids) {
                // Normal startup only auto-enters BluFi when the list was
                // empty; with saved SSIDs it tried to connect, so enter
                // provisioning explicitly now.
                auto* wifi = static_cast<WifiBoard*>(&GetCurrentBoard());
                wifi->EnterWifiConfigMode();
            }
        }
    }

    virtual void SetNetworkEventCallback(NetworkEventCallback callback) override {
        forward_callback_ = std::move(callback);
        DualNetworkBoard::SetNetworkEventCallback(
            [this](NetworkEvent event, const std::string& data) {
                if (event == NetworkEvent::Connected) {
                    // Every real Connected publishes modem readiness when
                    // appropriate, bumps the generation (forcing the queued
                    // apply), and wakes the control worker — it never touches
                    // apply state itself.
                    if (GetNetworkType() == NetworkType::ML307) {
                        modem_ready_.store(true, std::memory_order_release);
                    }
                    connection_generation_.fetch_add(1);
                    if (control_queue_ != nullptr && control_worker_ != nullptr) {
                        int token = 0;
                        xQueueOverwrite(control_queue_, &token);
                    }
                }
                if (forward_callback_) {
                    forward_callback_(event, data);
                }
            });
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        remembered_level_.store(static_cast<int>(level));
        if (control_queue_ != nullptr && control_worker_ != nullptr) {
            int token = 0;
            xQueueOverwrite(control_queue_, &token);
        }
    }

    // --- JSON ----------------------------------------------------------
    std::string BuildStateJson() {
        const auto power = peripherals_.PowerSnapshot();
        const auto motion = peripherals_.MotionSnapshot();
        const auto radar = peripherals_.RadarSnapshot();
        const auto inputs = peripherals_.InputSnapshot();

        cJSON* root = cJSON_CreateObject();
        cJSON* hw = cJSON_AddObjectToObject(root, "hardware");

        cJSON* p = cJSON_AddObjectToObject(hw, "power");
        AddCommonFields(p, power.available, power.valid, power.last_error, power.sample_us);
        // Every measured field is null when the snapshot is invalid.
        if (power.valid) {
            cJSON_AddBoolToObject(p, "battery_present", power.battery_present);
            cJSON_AddBoolToObject(p, "vbus_good", power.vbus_good);
            cJSON_AddBoolToObject(p, "charging", power.charging);
            cJSON_AddBoolToObject(p, "discharging", power.discharging);
            cJSON_AddBoolToObject(p, "charging_done", power.charging_done);
            AddNumberOrNull(p, "battery_percent", power.soc_valid, power.battery_percent);
            AddNumberOrNull(p, "battery_mv", power.battery_present, power.battery_mv);
            AddNumberOrNull(p, "vbus_mv", true, power.vbus_mv);
            AddNumberOrNull(p, "temperature_c", power.temperature_valid, power.temperature_c);
        } else {
            for (const char* k :
                 {"battery_present", "vbus_good", "charging", "discharging", "charging_done",
                  "battery_percent", "battery_mv", "vbus_mv", "temperature_c"}) {
                cJSON_AddNullToObject(p, k);
            }
        }

        cJSON* m = cJSON_AddObjectToObject(hw, "motion");
        AddCommonFields(m, motion.available, motion.valid, motion.last_error, motion.sample_us);
        cJSON* accel = cJSON_AddObjectToObject(m, "accel_mg");
        cJSON* gyro = cJSON_AddObjectToObject(m, "gyro_raw");
        if (motion.valid) {
            cJSON_AddNumberToObject(accel, "x", motion.accel_mg[0]);
            cJSON_AddNumberToObject(accel, "y", motion.accel_mg[1]);
            cJSON_AddNumberToObject(accel, "z", motion.accel_mg[2]);
            cJSON_AddNumberToObject(gyro, "x", motion.gyro_raw[0]);
            cJSON_AddNumberToObject(gyro, "y", motion.gyro_raw[1]);
            cJSON_AddNumberToObject(gyro, "z", motion.gyro_raw[2]);
        } else {
            for (const char* k : {"x", "y", "z"}) {
                cJSON_AddNullToObject(accel, k);
                cJSON_AddNullToObject(gyro, k);
            }
        }
        cJSON_AddNumberToObject(m, "gyro_ctrl3", 0x66);

        cJSON* r = cJSON_AddObjectToObject(hw, "radar");
        AddCommonFields(r, radar.available, radar.valid, radar.last_error, radar.sample_us);
        cJSON_AddBoolToObject(r, "version_received", radar.version_received);
        AddNumberOrNull(r, "state", radar.valid, radar.state);
        AddNumberOrNull(r, "distance_raw", radar.valid, radar.distance_raw);
        AddNumberOrNull(r, "signal_raw", radar.valid, radar.signal_raw);
        if (radar.version_received) {
            cJSON_AddStringToObject(r, "version", radar.version);
        } else {
            cJSON_AddNullToObject(r, "version");
        }

        cJSON* in = cJSON_AddObjectToObject(hw, "inputs");
        cJSON_AddNumberToObject(in, "button_gpio5", inputs.button);
        cJSON_AddNumberToObject(in, "radar_gpio16", inputs.radar_io);
        cJSON_AddNumberToObject(in, "imu_gpio42", inputs.imu_irq);

        char* text = cJSON_PrintUnformatted(root);
        std::string json = text != nullptr ? std::string(text) : std::string("{}");
        cJSON_free(text);
        cJSON_Delete(root);
        return json;
    }

    // Minimal identity/status before the modem backend is initialized —
    // same type/name/manufacturer shape as the backend implementations,
    // plus a structured network object; never dereferences the modem.
    static std::string MinimalJson() {
        return "{\"type\":\"" BOARD_TYPE "\",\"name\":\"" BOARD_NAME
               "\",\"manufacturer\":\"" BOARD_MANUFACTURER
               "\",\"network\":{\"type\":\"cellular\",\"ready\":false}}";
    }

    virtual std::string GetBoardJson() override {
        if (GetNetworkType() == NetworkType::ML307 && !modem_ready_.load()) {
            return MinimalJson();
        }
        return DualNetworkBoard::GetBoardJson();
    }

    virtual std::string GetDeviceStatusJson() override {
        std::string base;
        if (GetNetworkType() == NetworkType::ML307 && !modem_ready_.load()) {
            base = MinimalJson();
        } else {
            base = DualNetworkBoard::GetDeviceStatusJson();
        }
        cJSON* root = cJSON_Parse(base.c_str());
        if (root == nullptr) {
            return BuildStateJson();
        }
        cJSON* hw = cJSON_Parse(BuildStateJson().c_str());
        cJSON* hw_hw = hw != nullptr ? cJSON_DetachItemFromObject(hw, "hardware") : nullptr;
        if (hw_hw != nullptr) {
            cJSON_AddItemToObject(root, "hardware", hw_hw);
        }
        if (hw != nullptr) {
            cJSON_Delete(hw);
        }
        char* text = cJSON_PrintUnformatted(root);
        std::string out = text != nullptr ? std::string(text) : std::string();
        cJSON_free(text);
        cJSON_Delete(root);
        return out.empty() ? base : out;
    }
};

DECLARE_BOARD(XAtomBoard);
