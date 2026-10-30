"""Host tests for X-Atom helpers and production codec/peripheral paths.

Production bodies are extracted verbatim from the source files into a host
fixture with stubbed SDK calls, so tests run the real code. A subprocess
timeout catches any infinite parser/driver loop regression.
"""
import json
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
XATOM_DIR = ROOT / "main/boards/xatom"
CODEC_CC = ROOT / "main/audio/codecs/es8311_audio_codec.cc"
PERIPH_CC = XATOM_DIR / "xatom_peripherals.cc"
BOARD_CC = XATOM_DIR / "xatom_board.cc"
CJSON_C = ROOT / "managed_components/espressif__cjson/cJSON/cJSON.c"


def extract_function(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def extract_member(source, cls, signature):
    """Extract an in-class method body and emit an out-of-class definition
    qualified as cls::name, dropping virtual/static/override specifiers."""
    import re
    body = extract_function(source, signature)
    head, rest = body.split("{", 1)
    head = head.replace("virtual ", "").replace("static ", "")
    head = head.replace(" override", "").strip()
    paren = head.index("(")
    name_m = re.search(r"([A-Za-z_][A-Za-z0-9_]*)\s*$", head[:paren])
    head = head[:name_m.start(1)] + cls + "::" + name_m.group(1) + head[paren:]
    return head + "{" + rest


FIXTURE = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>
#include "main/boards/xatom/xatom_measurements.h"

// ---- shared SDK stubs ----------------------------------------------------
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
using TickType_t = uint32_t;
inline TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
#define ESP_ERROR_CHECK(x) do { esp_err_t _e = (x); if (_e != ESP_OK) return; } while (0)
#define ESP_ERROR_CHECK_WITHOUT_ABORT(x) do { if ((x) != ESP_OK) {} } while (0)
#define ESP_LOGI(...)
#define ESP_LOGE(...)
#define ESP_LOGW(...)

// ---- codec fixture -------------------------------------------------------
static TickType_t g_tick = 0;
TickType_t xTaskGetTickCount() { return g_tick; }
using esp_codec_dev_handle_t = void*;
constexpr int ESP_CODEC_DEV_TYPE_IN_OUT = 3;
struct audio_codec_data_if_t {};
struct audio_codec_ctrl_if_t {};
struct audio_codec_if_t {};
struct audio_codec_gpio_if_t {};
struct i2c_port_t_ {};
using i2c_port_t = int;
using gpio_num_t = int;
constexpr gpio_num_t GPIO_NUM_NC = -1;
struct esp_codec_dev_cfg_t {
    int dev_type;
    const audio_codec_if_t* codec_if;
    const audio_codec_data_if_t* data_if;
};
struct esp_codec_dev_sample_info_t {
    uint8_t bits_per_sample;
    uint8_t channel;
    uint32_t channel_mask;
    uint32_t sample_rate;
    int mclk_multiple;
};

static int g_read_rc = ESP_OK;
static int g_write_rc = ESP_OK;
static int g_read_calls = 0;
static int g_write_calls = 0;
static int g_fail_on_write_call = 0;
static int16_t g_written[2048];
static int g_written_len = 0;
static int g_last_fs_channel = -1;
static int g_open_calls = 0;
static int g_close_calls = 0;
static int g_pa_level = -1;
static esp_codec_dev_handle_t g_dev = (void*)0x42;

int esp_codec_dev_read(esp_codec_dev_handle_t, void*, int) {
    g_read_calls++;
    return g_read_rc;
}
int esp_codec_dev_write(esp_codec_dev_handle_t, const void* data, int bytes) {
    g_write_calls++;
    const int rc = (g_fail_on_write_call == g_write_calls) ? ESP_FAIL : g_write_rc;
    if (rc == ESP_OK) {
        memcpy(g_written + g_written_len, data, bytes);
        g_written_len += bytes / 2;
    }
    return rc;
}
esp_codec_dev_handle_t esp_codec_dev_new(const esp_codec_dev_cfg_t*) { return g_dev; }
esp_err_t esp_codec_dev_open(esp_codec_dev_handle_t, const esp_codec_dev_sample_info_t* fs) {
    g_open_calls++;
    g_last_fs_channel = fs->channel;
    return ESP_OK;
}
esp_err_t esp_codec_dev_close(esp_codec_dev_handle_t) {
    g_close_calls++;
    return ESP_OK;
}
esp_err_t esp_codec_dev_delete(esp_codec_dev_handle_t) { return ESP_OK; }
esp_err_t esp_codec_dev_set_in_gain(esp_codec_dev_handle_t, float) { return ESP_OK; }
esp_err_t esp_codec_dev_set_out_vol(esp_codec_dev_handle_t, int) { return ESP_OK; }
void audio_codec_delete_codec_if(const audio_codec_if_t*) {}
void audio_codec_delete_ctrl_if(const audio_codec_ctrl_if_t*) {}
void audio_codec_delete_gpio_if(const audio_codec_gpio_if_t*) {}
void audio_codec_delete_data_if(const audio_codec_data_if_t*) {}
esp_err_t gpio_set_level(gpio_num_t, uint32_t level) {
    g_pa_level = (int)level;
    return ESP_OK;
}

// Minimal AudioCodec base with the members the extracted bodies touch.
class AudioCodec {
public:
    bool duplex_ = false;
    bool input_reference_ = false;
    bool input_enabled_ = false;
    bool output_enabled_ = false;
    int input_sample_rate_ = 0;
    int output_sample_rate_ = 0;
    int input_channels_ = 1;
    int output_volume_ = 70;
    float input_gain_ = 30.0f;
    void* tx_handle_ = nullptr;
    void* rx_handle_ = nullptr;
    virtual ~AudioCodec() = default;
    virtual void EnableInput(bool enable) { input_enabled_ = enable; }
    virtual void EnableOutput(bool enable) { output_enabled_ = enable; }
    virtual void SetOutputVolume(int volume) { output_volume_ = volume; }
};

class Es8311AudioCodec : public AudioCodec {
public:
    const audio_codec_data_if_t* data_if_ = nullptr;
    const audio_codec_ctrl_if_t* ctrl_if_ = nullptr;
    const audio_codec_if_t* codec_if_ = nullptr;
    const audio_codec_gpio_if_t* gpio_if_ = nullptr;
    esp_codec_dev_handle_t dev_ = nullptr;
    gpio_num_t pa_pin_ = GPIO_NUM_NC;
    bool pa_inverted_ = false;
    std::mutex data_if_mutex_;
    std::atomic<TickType_t> reference_open_tick_{0};

    Es8311AudioCodec() : codec_if_(reinterpret_cast<const audio_codec_if_t*>(0x1)) {}
    void UpdateDeviceState();
    int Read(int16_t* dest, int samples);
    int Write(const int16_t* data, int samples);
    void EnableInput(bool enable);
    void EnableOutput(bool enable);
};

@@CODEC@@

// ---- codec tests ---------------------------------------------------------
static void test_legacy_mono() {
    Es8311AudioCodec c;
    c.input_reference_ = false;
    g_open_calls = g_close_calls = g_read_calls = g_write_calls = 0;
    c.EnableInput(true);
    c.EnableOutput(true);
    assert(g_last_fs_channel == 1 && g_open_calls == 1);
    int16_t buf[16];
    assert(c.Read(buf, 4) == 4 && g_read_calls == 1);
    int16_t tx[4] = {1, 2, 3, 4};
    g_write_rc = ESP_OK; g_written_len = 0;
    assert(c.Write(tx, 4) == 4 && g_write_calls == 1);
    assert(g_written_len == 4 && g_written[0] == 1 && g_written[3] == 4);
    // PA pin configured: disable output drives PA low.
    c.pa_pin_ = 41;
    c.EnableOutput(false);
    c.EnableInput(false);
    assert(g_pa_level == 0);
    assert(g_close_calls == 1);
}

static void test_reference_stereo() {
    Es8311AudioCodec c;
    c.input_reference_ = true;
    c.input_channels_ = 2;
    g_open_calls = g_close_calls = g_read_calls = 0;
    g_tick = 1000;
    c.EnableInput(true);
    c.EnableOutput(true);
    assert(g_last_fs_channel == 2 && g_open_calls == 1);
    // settling: reopening via UpdateDeviceState resets the window
    g_tick = 1000 + 999;
    int16_t buf[16];
    assert(c.Read(buf, 4) == 0 && g_read_calls == 0);
    g_tick = 1000 + 1000;
    assert(c.Read(buf, 4) == 4 && g_read_calls == 1);
    // disable + reopen restarts settling through real UpdateDeviceState
    c.EnableInput(false);
    c.EnableOutput(false);
    g_open_calls = 0; g_read_calls = 0;
    g_tick = 5000;
    c.EnableInput(true);
    assert(g_open_calls == 1);
    g_tick = 5000 + 500;
    assert(c.Read(buf, 4) == 0);
    g_tick = 5000 + 1000;
    assert(c.Read(buf, 4) == 4);
    // failed read returns 0
    g_read_rc = ESP_FAIL;
    assert(c.Read(buf, 4) == 0);
    g_read_rc = ESP_OK;
}

static void test_reference_duplication() {
    for (int n : {3, 240, 241}) {
        Es8311AudioCodec c;
        c.input_reference_ = true;
        c.input_enabled_ = c.output_enabled_ = true;
        int16_t tx[512];
        for (int i = 0; i < n; i++) tx[i] = i * 2 - 100;
        g_write_rc = ESP_OK; g_write_calls = 0; g_written_len = 0;
        assert(c.Write(tx, n) == n);
        assert(g_written_len == n * 2);
        for (int i = 0; i < n; i++) {
            assert(g_written[i * 2] == tx[i] && g_written[i * 2 + 1] == tx[i]);
        }
    }
    Es8311AudioCodec c;
    c.input_reference_ = true;
    c.input_enabled_ = c.output_enabled_ = true;
    int16_t tx[500];
    for (int i = 0; i < 500; i++) tx[i] = i;
    g_write_calls = 0; g_written_len = 0;
    g_write_rc = ESP_OK;
    g_fail_on_write_call = 2;
    assert(c.Write(tx, 241) == 240);  // second chunk failed -> mono prefix
    assert(g_written_len == 240 * 2);
    g_fail_on_write_call = 0;
    g_write_rc = ESP_FAIL; g_write_calls = 0; g_written_len = 0;
    assert(c.Write(tx, 8) == 0);
    g_write_rc = ESP_OK;
}

// ---- PMIC / IMU ----------------------------------------------------------
static void test_pmic() {
    xatom::PmicSnapshot s{};
    assert(xatom::PmicDecode(0x28, 0x14, 0x10, 0x52, 0x13, 0xF3, 0x1C, 0x6A, 100, &s));
    assert(s.battery_present && s.vbus_good);
    assert(s.charging_done && !s.charging && !s.discharging);
    assert(s.battery_mv == 4178 && s.vbus_mv == 5107);
    assert(s.temperature_valid && s.temperature_c > 21.99f && s.temperature_c < 22.01f);
    assert(s.soc_valid && s.battery_percent == 100);
    s.valid = true;
    assert(xatom::PmicBatteryLevelValid(s));
    // float boundaries: raw 7273 -> 22.05C; raw 8515 -> -40.05C invalid;
    // raw 5213 -> 125.05C invalid.
    xatom::PmicSnapshot c{};
    xatom::PmicDecode(0x28, 0x14, 0, 0, 0, 0, 0x1C, 0x69, 0, &c);
    assert(c.temperature_valid && c.temperature_c > 22.04f && c.temperature_c < 22.06f);
    xatom::PmicDecode(0x28, 0x14, 0, 0, 0, 0, 0x21, 0x43, 0, &c);
    assert(!c.temperature_valid);  // raw 8515
    xatom::PmicDecode(0x28, 0x14, 0, 0, 0, 0, 0x14, 0x5D, 0, &c);
    assert(!c.temperature_valid);  // raw 5213
    xatom::PmicDecode(0x28, 0x20, 0, 0, 0, 0, 0, 0, 0, &c);
    assert(c.charging && !c.discharging);
    xatom::PmicDecode(0x28, 0x40, 0, 0, 0, 0, 0, 0, 0, &c);
    assert(c.discharging && !c.charging);
    xatom::PmicDecode(0x28, 0x14, 0, 0, 0, 0, 0, 0, 255, &c);
    assert(!c.soc_valid);
    xatom::PmicDecode(0x00, 0x14, 0, 0, 0, 0, 0, 0, 50, &c);
    assert(!c.soc_valid);
    xatom::PmicDecode(0x28, 0x14, 0, 0, 0, 0, 0x00, 0x00, 0, &c);
    assert(!c.temperature_valid);
    assert(xatom::PmicDecode(0, 0, 0, 0, 0, 0, 0, 0, 0, nullptr) == false);
}

static void test_imu() {
    const uint8_t b[12] = {0x00, 0x00, 0x00, 0x10, 0x00, 0xF0,
                           0xFF, 0x7F, 0x00, 0x80, 0xFF, 0xFF};
    xatom::ImuSample s{};
    assert(xatom::ImuDecode(b, 12, &s));
    assert(s.accel_mg[0] == 0.0f && s.accel_mg[1] == 1000.0f &&
           s.accel_mg[2] == -1000.0f);
    assert(s.gyro_raw[0] == 32767 && s.gyro_raw[1] == -32768 && s.gyro_raw[2] == -1);
    // unit conversion: raw +1/-1 -> +/-0.244140625 mg
    const uint8_t u[12] = {0x01, 0x00, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0};
    xatom::ImuDecode(u, 12, &s);
    assert(s.accel_mg[0] == 0.244140625f && s.accel_mg[1] == -0.244140625f);
    assert(!xatom::ImuDecode(b, 8, &s));
}

// ---- radar parser (Append/Next streaming) --------------------------------
static void feed_all(xatom::XAtomRadarParser& p, const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; i++) p.Append(d[i]);
}
static void test_radar() {
    const uint8_t ver[] = {0x55, 0xA5, 0x00, 0x08, 0x02, 0x00,
                           0x01, 0x54, 0x45, 0x53, 0x54, 0x45};
    const uint8_t tel[] = {0x55, 0xA5, 0x00, 0x0E, 0x03, 0x81, 0x00, 0x00, 0x01,
                           0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x97};
    xatom::XAtomRadarParser p;
    xatom::XAtomRadarParser::Frame f;
    feed_all(p, ver, sizeof(ver));
    assert(p.Next(&f) && !p.Next(&f));
    char v[48];
    assert(xatom::RadarParseVersion(f, v, sizeof(v)) && strcmp(v, "TEST") == 0);

    p.Reset();
    feed_all(p, tel, sizeof(tel));
    assert(p.Next(&f));
    xatom::RadarTelemetry t;
    assert(xatom::RadarParseTelemetry(f, &t));
    assert(t.state == 1 && t.distance_raw == 258 && t.signal_raw == 772);

    // split feed across two appends sequences
    p.Reset();
    feed_all(p, ver, 5);
    assert(!p.Next(&f));
    feed_all(p, ver + 5, sizeof(ver) - 5);
    assert(p.Next(&f));

    // noise + bad checksum, then good frame recovered
    p.Reset();
    const uint8_t noise[] = {0x00, 0x11, 0x22};
    const uint8_t bad[] = {0x55, 0xA5, 0x00, 0x08, 0x02, 0x00,
                           0x01, 0x54, 0x45, 0x53, 0x54, 0x99};
    feed_all(p, noise, sizeof(noise));
    assert(!p.Next(&f));
    feed_all(p, bad, sizeof(bad));
    assert(!p.Next(&f));
    feed_all(p, ver, sizeof(ver));
    assert(p.Next(&f));

    // bad length dropped, later frame recovered
    p.Reset();
    const uint8_t badlen[] = {0x55, 0xA5, 0xFF, 0xFF, 0x01, 0x02};
    feed_all(p, badlen, sizeof(badlen));
    assert(!p.Next(&f));
    feed_all(p, ver, sizeof(ver));
    assert(p.Next(&f));

    // coalesced: one Append stream, two Next extractions
    p.Reset();
    uint8_t both[sizeof(ver) * 2];
    memcpy(both, ver, sizeof(ver));
    memcpy(both + sizeof(ver), ver, sizeof(ver));
    feed_all(p, both, sizeof(both));
    assert(p.Next(&f));
    assert(p.Next(&f));
    assert(xatom::RadarParseVersion(f, v, sizeof(v)) && strcmp(v, "TEST") == 0);
    assert(!p.Next(&f));

    // version payload must be nonempty printable ASCII
    p.Reset();
    xatom::XAtomRadarParser::Frame badver = f;
    badver.func = 2; badver.cmd = 1; badver.payload_len = 0;
    assert(!xatom::RadarParseVersion(badver, v, sizeof(v)));
    badver.payload_len = 2; badver.payload[0] = 0x01; badver.payload[1] = 0x00;
    assert(!xatom::RadarParseVersion(badver, v, sizeof(v)));
}

// ---- production DrainRadar driver fixture --------------------------------
static std::vector<uint8_t> g_uart_rx;
static int g_uart_reads = 0;
static int g_dispatches = 0;
int uart_read_bytes(int, uint8_t* buf, uint32_t len, uint32_t) {
    g_uart_reads++;
    if (g_uart_rx.empty()) return 0;
    const size_t n = std::min<size_t>(len, g_uart_rx.size());
    memcpy(buf, g_uart_rx.data(), n);
    g_uart_rx.erase(g_uart_rx.begin(), g_uart_rx.begin() + n);
    return (int)n;
}
constexpr int XATOM_RADAR_UART = 2;
constexpr int kRadarChunk = 128;

struct XAtomRadarSnapshot {
    bool available = false;
    bool valid = false;
};

static int g_last_func = -1, g_last_cmd = -1;
static size_t g_last_payload_len = 0;
static std::vector<size_t> g_payload_lens;

class XAtomPeripherals {
public:
    XAtomRadarSnapshot radar_;
    xatom::XAtomRadarParser parser_;
    void DrainRadar();
    void HandleRadarFrame(const xatom::XAtomRadarParser::Frame& f) {
        g_dispatches++;
        g_last_func = f.func;
        g_last_cmd = f.cmd;
        g_last_payload_len = f.payload_len;
        g_payload_lens.push_back(f.payload_len);
    }
};

@@DRAIN@@

static void test_drain_radar_two_frames() {
    const uint8_t ver[] = {0x55, 0xA5, 0x00, 0x08, 0x02, 0x00,
                           0x01, 0x54, 0x45, 0x53, 0x54, 0x45};
    const uint8_t tel[] = {0x55, 0xA5, 0x00, 0x0E, 0x03, 0x81, 0x00, 0x00, 0x01,
                           0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x97};
    XAtomPeripherals d;
    d.radar_.available = true;
    g_uart_rx.assign(ver, ver + sizeof(ver));
    g_uart_rx.insert(g_uart_rx.end(), tel, tel + sizeof(tel));
    g_dispatches = 0;
    d.DrainRadar();  // one UART buffer, two different frames
    assert(g_dispatches == 2);
    d.DrainRadar();  // nothing left: bounded return, no replay
    assert(g_dispatches == 2);

    // noise + bad checksum + good frame in one buffer -> exactly 1 dispatch
    g_uart_rx = {0x00, 0x11, 0x55, 0xA5, 0x00, 0x08, 0x02, 0x00, 0x01, 0x54,
                 0x45, 0x53, 0x54, 0x99};
    g_uart_rx.insert(g_uart_rx.end(), ver, ver + sizeof(ver));
    g_dispatches = 0;
    d.DrainRadar();
    assert(g_dispatches == 1);

    // 256-byte frame spanning 128/128/24 UART chunks: noise + large frame +
    // version reply; exactly one dispatch each, payload 248.
    std::vector<uint8_t> big = {0x55, 0xA5, 0x00, 0xFC, 0x07, 0x12, 0x34};
    big.insert(big.end(), 248, 0xAB);
    uint8_t sum = 0;
    for (uint8_t b : big) sum += b;
    big.push_back(sum);  // 0xEB
    g_uart_rx.assign(12, 0x99);  // noise prefix
    g_uart_rx.insert(g_uart_rx.end(), big.begin(), big.end());
    g_uart_rx.insert(g_uart_rx.end(), ver, ver + sizeof(ver));
    g_dispatches = 0;
    g_payload_lens.clear();
    while (!g_uart_rx.empty()) {
        d.DrainRadar();  // chunks of <=128 per read
    }
    d.DrainRadar();
    assert(g_dispatches == 2);
    assert(g_payload_lens[0] == 248);  // large frame first, version second
    assert(g_last_func == 2 && g_last_cmd == 1 && g_last_payload_len == 4);
}

static void test_disabled_reference_io() {
    // Reference mode: disabled Read/Write transfer 0 logical frames and make
    // zero codec SDK calls; mono keeps the legacy "pretend success" return.
    Es8311AudioCodec c;
    c.input_reference_ = true;
    int16_t buf[8];
    g_read_calls = g_write_calls = 0;
    assert(c.Read(buf, 8) == 0 && g_read_calls == 0);
    int16_t tx[8] = {};
    assert(c.Write(tx, 8) == 0 && g_write_calls == 0);
    Es8311AudioCodec m;
    m.input_reference_ = false;
    assert(m.Read(buf, 8) == 8 && m.Write(tx, 8) == 8);
    assert(g_read_calls == 0 && g_write_calls == 0);
}

int main() {
    test_legacy_mono();
    test_reference_stereo();
    test_reference_duplication();
    test_disabled_reference_io();
    test_pmic();
    test_imu();
    test_radar();
    test_drain_radar_two_frames();
    return 0;
}
'''


# Production board/peripheral surfaces with real cJSON linked in.
FIXTURE2 = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <variant>
#include <vector>
#include <cJSON.h>
#include "main/boards/xatom/xatom_measurements.h"

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr esp_err_t ESP_ERR_INVALID_SIZE = -2;
constexpr esp_err_t ESP_ERR_INVALID_RESPONSE = -3;
constexpr esp_err_t ESP_ERR_INVALID_STATE = -4;
using TickType_t = uint32_t;
using gpio_num_t = int;
#define ESP_LOGI(...)
#define ESP_LOGE(...)
#define ESP_LOGW(...)

static int64_t g_now = 0;
int64_t esp_timer_get_time() { return g_now; }
const char* esp_err_to_name(esp_err_t) { return "err"; }

// snapshot types matching the production layout used by the extracted code
struct XAtomRadarSnapshot {
    bool available = false;
    bool valid = false;
    bool version_received = false;
    int last_error = 0;
    int64_t sample_us = 0;
    uint8_t state = 0;
    uint16_t distance_raw = 0;
    uint16_t signal_raw = 0;
    char version[48] = {};
    int io_level = 0;
};
struct XAtomInputSnapshot {
    int button = 1;
    int radar_io = 0;
    int imu_irq = 0;
};

enum class NetworkType { WIFI, ML307 };
enum class NetworkEvent { Scanning, Connecting, Connected, Disconnected };
enum class PowerSaveLevel { LOW_POWER, BALANCED, PERFORMANCE };
using NetworkEventCallback = std::function<void(NetworkEvent, const std::string&)>;

using QueueHandle_t = struct FakeQ*;
struct FakeQ { int v = 0; bool full = false; };
using TaskHandle_t = void*;
using esp_timer_handle_t = void*;

static bool g_timer_ok = true;
esp_err_t esp_timer_start_once(esp_timer_handle_t, uint64_t) {
    return g_timer_ok ? ESP_OK : ESP_FAIL;
}
int xQueueOverwrite(QueueHandle_t q, const int* v) {
    if (q) { q->v = *v; q->full = true; }
    return 1;
}
int xQueueReceive(QueueHandle_t q, int* v, uint32_t) {
    if (q == nullptr || !q->full) return 0;  // pdFALSE ends the worker loop
    *v = q->v; q->full = false;
    return 1;
}

// ---- application / settings / backend stubs ------------------------------
static std::vector<std::function<void()>> g_app_queue;
static int g_reboots = 0;
static int g_dirty_settings = 0;
struct Commit { std::string ns, key; int i = 0; bool has_int = false, b = false; };
static std::vector<Commit> g_committed;
static bool g_flag_provision = false;
class Settings {
public:
    Settings(const char* ns, bool rw) : ns_(ns), rw_(rw) {}
    ~Settings() {
        if (dirty_) {
            for (auto& c : pending_) g_committed.push_back(c);
            g_dirty_settings--;
        }
    }
    int GetInt(const char*, int d) { return d; }
    bool GetBool(const char*, bool d) {
        return ns_ == "xatom" ? g_flag_provision : d;
    }
    void SetInt(const char* k, int v) {
        MarkDirty();
        pending_.push_back({ns_, k, v, true, false});
    }
    void SetBool(const char* k, bool v) {
        MarkDirty();
        pending_.push_back({ns_, k, 0, false, v});
    }
private:
    void MarkDirty() {
        if (rw_ && !dirty_) { dirty_ = true; g_dirty_settings++; }
    }
    std::string ns_;
    bool rw_;
    bool dirty_ = false;
    std::vector<Commit> pending_;
};

class Application {
public:
    static Application& GetInstance() { static Application a; return a; }
    void Schedule(std::function<void()> f) { g_app_queue.push_back(std::move(f)); }
    void Reboot() {
        // Persistent writes must be committed (Settings destructed) before
        // reboot: a dirty owner still alive here is the reported bug.
        assert(g_dirty_settings == 0);
        g_reboots++;
    }
};
static void RunAppQueue() {
    auto q = std::move(g_app_queue);
    g_app_queue.clear();
    for (auto& f : q) f();
}

class WifiBoard;
class Board;
static Board* g_board = nullptr;
class Board {
public:
    static Board& GetInstance() { return *g_board; }
};

class PeripheralsStub {
public:
    xatom::PmicSnapshot pmic;
    xatom::ImuSample imu;
    XAtomRadarSnapshot radar;
    XAtomInputSnapshot inputs;
    int wake_acks = 0;
    xatom::PmicSnapshot PowerSnapshot() { return pmic; }
    xatom::ImuSample MotionSnapshot() { return imu; }
    XAtomRadarSnapshot RadarSnapshot() { return radar; }
    XAtomInputSnapshot InputSnapshot() { return inputs; }
    void WakeAcknowledged() { wake_acks++; }
};

class PowerSaveTimer {
public:
    int wakes = 0;
    void WakeUp() { wakes++; }
};

static int g_at_calls = 0;
static bool g_at_rc = true;
static std::function<void()> g_at_hook;
class AtModem {
public:
    bool SetSleepMode(bool, int) {
        g_at_calls++;
        if (g_at_hook) { auto h = g_at_hook; g_at_hook = nullptr; h(); }
        return g_at_rc;
    }
};

class DualNetworkBoard : public Board {
public:
    NetworkType net_type = NetworkType::ML307;
    AtModem modem;
    NetworkEventCallback backend_cb;
    int wifi_apply_calls = 0;
    PowerSaveLevel wifi_level = PowerSaveLevel::PERFORMANCE;
    virtual ~DualNetworkBoard() = default;
    virtual void SetPowerSaveLevel(PowerSaveLevel l) {
        wifi_apply_calls++;
        wifi_level = l;
    }
    virtual void SetNetworkEventCallback(NetworkEventCallback cb) {
        backend_cb = std::move(cb);
    }
    virtual std::string GetBoardJson() { return "{}"; }
    virtual std::string GetDeviceStatusJson() { return "{}"; }
    NetworkType GetNetworkType() { return net_type; }
    void* GetNetwork() { return &modem; }
    Board& GetCurrentBoard() { return *this; }
};

static constexpr int kActionNone = 0;
static constexpr int kActionShutdown = 1;
static constexpr int kActionSwitchToWifi = 2;
static constexpr int kActionSwitchTo4g = 3;
static constexpr int kActionWifiConfig = 4;
static constexpr int kActionRescueWifi = 5;
static constexpr int kActionReboot = 6;
static const char* kProvisionKey = "provision";

static int g_shutdown_calls = 0;
static int g_wifi_config_calls = 0;

// forward declarations needed by the extracted member bodies below
using i2c_master_dev_handle_t = void*;
constexpr int XATOM_RADAR_UART = 2;
constexpr uint32_t portMAX_DELAY = 0;
constexpr int pdTRUE = 1;
void AddCommonFields(cJSON* obj, bool available, bool valid, int last_error,
                     int64_t sample_us);
void AddNumberOrNull(cJSON* obj, const char* key, bool valid, double value);
static esp_err_t RegReadRc(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out,
                           size_t len);

class XAtomBoard : public DualNetworkBoard {
public:
    PeripheralsStub peripherals_;
    PowerSaveTimer* power_save_timer_ = nullptr;
    esp_timer_handle_t activity_timer_ = (void*)1;
    esp_timer_handle_t action_timer_ = (void*)1;
    std::atomic<bool> activity_pending_{false};
    std::atomic<int> pending_action_{kActionNone};
    std::atomic<bool> modem_ready_{false};
    std::atomic<int> remembered_level_{static_cast<int>(PowerSaveLevel::PERFORMANCE)};
    std::atomic<uint32_t> connection_generation_{0};
    int last_applied_level_ = -1;
    uint32_t last_applied_generation_ = UINT32_MAX;
    NetworkEventCallback forward_callback_;
    QueueHandle_t control_queue_ = nullptr;
    TaskHandle_t control_worker_ = nullptr;
    FakeQ fake_queue;
    void DoShutdown() { g_shutdown_calls++; }
    void EnterWifiConfig() { g_wifi_config_calls++; }

    void RequestActivity();
    static void ActivityTimerCallback(void* arg);
    bool QueueAction(int action);
    static void ActionTimerCallback(void* arg);
    void ControlWorkerLoop();
    void ApplyPowerSaveLevel(PowerSaveLevel level, uint32_t generation);
    void SetPowerSaveLevel(PowerSaveLevel level) override;
    void SetNetworkEventCallback(NetworkEventCallback callback) override;
    std::string BuildStateJson();
    static std::string MinimalJson();
    std::string GetBoardJson() override;
    std::string GetDeviceStatusJson() override;
    bool GetTemperature(float& celsius);
};
class WifiBoard : public DualNetworkBoard {
public:
    void EnterWifiConfigMode() { g_wifi_config_calls++; }
};
@@BOARD@@

// ---- I2C + UART stubs for peripheral bodies -------------------------------
static std::vector<esp_err_t> g_i2c_rcs;
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t*,
                                      size_t, uint8_t* out, size_t len, int) {
    esp_err_t rc = ESP_OK;
    if (!g_i2c_rcs.empty()) {
        rc = g_i2c_rcs.front();
        g_i2c_rcs.erase(g_i2c_rcs.begin());
    }
    if (rc == ESP_OK) {
        static const uint8_t resp[] = {0x28, 0x14, 0x10, 0x52, 0x13, 0xF3,
                                       0x1C, 0x6A, 100};
        static int idx = 0;
        for (size_t i = 0; i < len; i++) out[i] = resp[idx++ % sizeof(resp)];
    }
    return rc;
}
static int g_uart_writes = 0;
static int g_uart_write_rc = 0;
int uart_write_bytes(int, const void*, size_t len) {
    g_uart_writes++;
    return g_uart_write_rc >= 0 ? (g_uart_write_rc == 0 ? (int)len : g_uart_write_rc) : g_uart_write_rc;
}

struct RadarSnapW { bool available = false; int last_error = 0; std::mutex m; };

class XAtomPeripherals {
public:
    RadarSnapW radar_;
    int version_attempts_ = 0;
    std::mutex mutex_;
    void SendRadarVersionRequest();
    void SamplePmic();
    i2c_master_dev_handle_t pmic_dev_ = (void*)1;
    xatom::PmicSnapshot pmic_;
};

@@PERIPH@@

// production helper used by both extracted bodies
@@REGREAD@@

@@HELPERS@@

// ---- tests ----------------------------------------------------------------
static void test_power_policy() {
    XAtomBoard b;
    g_board = &b;
    b.net_type = NetworkType::ML307;
    b.modem_ready_.store(true);
    b.control_queue_ = &b.fake_queue;
    b.control_worker_ = (void*)1;
    b.SetNetworkEventCallback(NetworkEventCallback{});  // installs backend_cb
    // failed apply is not recorded: retry of the same level runs again
    g_at_rc = false; g_at_calls = 0;
    b.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    b.ControlWorkerLoop();
    assert(g_at_calls == 1);
    g_at_rc = true;
    b.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    b.ControlWorkerLoop();
    assert(g_at_calls == 2);
    // identical success dedups
    b.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    b.ControlWorkerLoop();
    assert(g_at_calls == 2);
    // every Connected forces an apply
    b.backend_cb(NetworkEvent::Connected, "");
    b.ControlWorkerLoop();
    assert(g_at_calls == 3);
    // Connected during a blocking AT -> its token applies with the bumped
    // generation inside the same worker pass: two AT calls total here.
    g_at_hook = [&b]() { b.backend_cb(NetworkEvent::Connected, ""); };
    b.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    b.ControlWorkerLoop();
    assert(g_at_calls == 5);
    // absent worker/queue is a no-op
    XAtomBoard b2;
    b2.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    assert(!b2.fake_queue.full);
    // unready modem: no pointer access, defer
    XAtomBoard b3;
    b3.net_type = NetworkType::ML307;
    b3.control_queue_ = &b3.fake_queue;
    b3.control_worker_ = (void*)1;
    g_at_calls = 0;
    b3.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    b3.ControlWorkerLoop();
    assert(g_at_calls == 0);
    // wi-fi path applies through base
    XAtomBoard b4;
    b4.net_type = NetworkType::WIFI;
    b4.control_queue_ = &b4.fake_queue;
    b4.control_worker_ = (void*)1;
    b4.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
    b4.ControlWorkerLoop();
    assert(b4.wifi_apply_calls == 1 && b4.wifi_level == PowerSaveLevel::LOW_POWER);
}

static void test_actions() {
    XAtomBoard b;
    g_board = &b;
    // switch target captured: 4g -> writes type=1 then Reboot
    assert(b.QueueAction(kActionSwitchTo4g));
    assert(!b.QueueAction(kActionShutdown));  // second pending rejected
    XAtomBoard::ActionTimerCallback(&b);
    assert(b.pending_action_.load() != kActionNone);  // latch held until cb
    assert(!b.QueueAction(kActionWifiConfig));  // still pending while queued
    RunAppQueue();
    assert(b.pending_action_.load() == kActionNone);
    assert(g_reboots == 1);  // reboot happened only after commits
    bool saw = false;
    for (auto& s : g_committed)
        if (s.ns == "network" && s.key == "type" && s.i == 1) saw = true;
    assert(saw);
    // wifi target commits type=0 before Reboot
    g_committed.clear();
    assert(b.QueueAction(kActionSwitchToWifi));
    XAtomBoard::ActionTimerCallback(&b);
    RunAppQueue();
    saw = false;
    for (auto& s : g_committed)
        if (s.ns == "network" && s.key == "type" && s.i == 0) saw = true;
    assert(saw && g_reboots == 2);
    // rescue: type=0 + provision flag committed before Reboot
    g_committed.clear();
    assert(b.QueueAction(kActionRescueWifi));
    XAtomBoard::ActionTimerCallback(&b);
    RunAppQueue();
    bool saw0 = false, sawflag = false;
    for (auto& s : g_committed) {
        if (s.ns == "network" && s.key == "type" && s.i == 0) saw0 = true;
        if (s.ns == "xatom" && s.key == "provision" && s.b) sawflag = true;
    }
    assert(saw0 && sawflag && g_reboots == 3);
    // shutdown path calls DoShutdown
    assert(b.QueueAction(kActionShutdown));
    XAtomBoard::ActionTimerCallback(&b);
    RunAppQueue();
    assert(g_shutdown_calls == 1);
    // timer-start failure releases latch immediately
    g_timer_ok = false;
    assert(!b.QueueAction(kActionShutdown));
    assert(b.pending_action_.load() == kActionNone);
    g_timer_ok = true;
}

static void test_activity() {
    XAtomBoard b;
    PowerSaveTimer pst;
    b.power_save_timer_ = &pst;
    b.RequestActivity();
    assert(b.activity_pending_.load());
    XAtomBoard::ActivityTimerCallback(&b);
    assert(pst.wakes == 1 && b.peripherals_.wake_acks == 1);
    // null timer still releases the radar latch
    XAtomBoard b2;
    b2.power_save_timer_ = nullptr;
    b2.RequestActivity();
    XAtomBoard::ActivityTimerCallback(&b2);
    assert(b2.peripherals_.wake_acks == 1);
    // timer failure clears pending and acknowledges
    XAtomBoard b3;
    b3.power_save_timer_ = &pst;
    g_timer_ok = false;
    b3.RequestActivity();
    assert(!b3.activity_pending_.load() && b3.peripherals_.wake_acks == 1);
    g_timer_ok = true;
}

static void test_version_requests() {
    XAtomPeripherals p;
    p.radar_.available = false;
    p.SendRadarVersionRequest();
    assert(g_uart_writes == 0 && p.version_attempts_ == 0);
    p.radar_.available = true;
    g_uart_write_rc = -7;  // negative -> actual negative error kept
    p.SendRadarVersionRequest();
    p.SendRadarVersionRequest();
    p.SendRadarVersionRequest();
    assert(g_uart_writes == 3 && p.version_attempts_ == 3);
    assert(p.radar_.last_error == -7);
    g_uart_write_rc = 5;  // partial -> ESP_ERR_INVALID_SIZE
    p.SendRadarVersionRequest();
    assert(p.radar_.last_error == ESP_ERR_INVALID_SIZE);
}

static void test_sample_pmic() {
    XAtomPeripherals p;
    p.pmic_.available = true;
    g_i2c_rcs.clear();
    g_now = 1234;
    p.SamplePmic();
    assert(p.pmic_.valid && p.pmic_.last_error == 0);
    assert(p.pmic_.sample_us == 1234);
    g_i2c_rcs.assign(5, ESP_ERR_INVALID_STATE);
    p.SamplePmic();
    assert(!p.pmic_.valid && p.pmic_.last_error == ESP_ERR_INVALID_STATE);
}

int main() {
    test_power_policy();
    test_actions();
    test_activity();
    test_version_requests();
    test_sample_pmic();
    // emit JSON for python-side validation
    XAtomBoard b;
    g_board = &b;
    b.net_type = NetworkType::ML307;  // unready -> MinimalJson
    printf("MINIMAL:%s\n", b.MinimalJson().c_str());
    printf("BOARD:%s\n", b.GetBoardJson().c_str());
    // invalid snapshot -> all null
    printf("STATEINV:%s\n", b.BuildStateJson().c_str());
    // valid power + invalid motion + version-only radar
    b.peripherals_.pmic.available = b.peripherals_.pmic.valid = true;
    b.peripherals_.pmic.battery_present = true;
    b.peripherals_.pmic.battery_mv = 4178;
    b.peripherals_.pmic.soc_valid = true;
    b.peripherals_.pmic.battery_percent = 100;
    b.peripherals_.pmic.temperature_valid = true;
    b.peripherals_.pmic.temperature_c = 22.0f;
    b.peripherals_.radar.available = true;
    b.peripherals_.radar.version_received = true;
    strcpy(b.peripherals_.radar.version, "TEST");
    printf("STATEMIX:%s\n", b.BuildStateJson().c_str());
    // failed SDK read: stale values stay cached but JSON publishes null +
    // exact error code
    b.peripherals_.pmic.valid = false;
    b.peripherals_.pmic.last_error = ESP_ERR_INVALID_STATE;
    printf("STATEERR:%s\n", b.BuildStateJson().c_str());
    // GetTemperature: invalid snapshot leaves sentinel untouched; valid sets
    float t = 123.0f;
    b.peripherals_.pmic.valid = false;
    b.peripherals_.pmic.temperature_valid = false;
    assert(!b.GetTemperature(t) && t == 123.0f);
    b.peripherals_.pmic.valid = true;
    b.peripherals_.pmic.temperature_valid = true;
    b.peripherals_.pmic.temperature_c = 22.05f;
    assert(b.GetTemperature(t) && t == 22.05f);
    b.peripherals_.pmic.valid = true;
    b.peripherals_.pmic.temperature_valid = false;
    assert(!b.GetTemperature(t) && t == 22.05f);
    return 0;
}
'''


@unittest.skipUnless(shutil.which("c++"), "requires a host C++ compiler")
class XAtomTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        codec = CODEC_CC.read_text()
        codec_bodies = "\n".join(
            extract_function(codec, sig)
            for sig in ("int Es8311AudioCodec::Read(", "int Es8311AudioCodec::Write(",
                        "void Es8311AudioCodec::UpdateDeviceState()",
                        "void Es8311AudioCodec::EnableInput(",
                        "void Es8311AudioCodec::EnableOutput("))
        periph = PERIPH_CC.read_text()
        drain = extract_function(periph, "void XAtomPeripherals::DrainRadar()")
        src = (FIXTURE.replace("@@CODEC@@", codec_bodies)
               .replace("@@DRAIN@@", drain))
        src_path = pathlib.Path(cls.tmp.name) / "xatom.cc"
        src_path.write_text(src)
        cls.binary = str(pathlib.Path(cls.tmp.name) / "xatom")
        subprocess.run(
            ["c++", "-std=c++17", "-Wall", "-Werror", "-I", str(ROOT),
             str(src_path), "-o", cls.binary],
            check=True, capture_output=True, text=True)

        # Second fixture: board/peripheral production bodies + real cJSON.
        board = BOARD_CC.read_text()
        periph = PERIPH_CC.read_text()
        member = lambda cls, sig: extract_member(board, cls, sig)
        board_bodies = "\n".join([
            member("XAtomBoard", "void RequestActivity()"),
            member("XAtomBoard", "static void ActivityTimerCallback(void* arg)"),
            member("XAtomBoard", "bool QueueAction(int action)"),
            member("XAtomBoard", "static void ActionTimerCallback(void* arg)"),
            member("XAtomBoard", "void ControlWorkerLoop()"),
            member("XAtomBoard",
                   "void ApplyPowerSaveLevel(PowerSaveLevel level, uint32_t generation)"),
            member("XAtomBoard",
                   "virtual void SetPowerSaveLevel(PowerSaveLevel level) override"),
            member("XAtomBoard",
                   "virtual void SetNetworkEventCallback(NetworkEventCallback callback) override"),
            member("XAtomBoard", "std::string BuildStateJson()"),
            member("XAtomBoard", "static std::string MinimalJson()"),
            member("XAtomBoard", "virtual std::string GetBoardJson() override"),
            member("XAtomBoard", "virtual std::string GetDeviceStatusJson() override"),
            member("XAtomBoard",
                   "virtual bool GetTemperature(float& celsius) override"),
        ])
        periph_bodies = "\n".join(
            extract_function(periph, sig)
            for sig in ("void XAtomPeripherals::SamplePmic()",
                        "void XAtomPeripherals::SendRadarVersionRequest()"))
        regread = extract_function(periph, "static esp_err_t RegReadRc(")
        helpers = "\n".join(
            extract_function(board, sig)
            for sig in ("void AddCommonFields(cJSON*", "void AddNumberOrNull(cJSON*"))
        src2 = (FIXTURE2.replace("@@BOARD@@", board_bodies)
                .replace("@@PERIPH@@", periph_bodies)
                .replace("@@REGREAD@@", regread)
                .replace("@@HELPERS@@", helpers))
        src2_path = pathlib.Path(cls.tmp.name) / "xatom_board.cc"
        src2_path.write_text(src2)
        cls.binary2 = str(pathlib.Path(cls.tmp.name) / "xatom_board")
        cjson_obj = str(pathlib.Path(cls.tmp.name) / "cjson.o")
        subprocess.run(["cc", "-std=c99", "-I", str(CJSON_C.parent), "-c",
                        str(CJSON_C), "-o", cjson_obj],
                       check=True, capture_output=True, text=True)
        subprocess.run(
            ["c++", "-std=c++17", "-Wall", "-Werror", "-I", str(ROOT),
             "-I", str(CJSON_C.parent),
             '-DBOARD_TYPE="xatom"', '-DBOARD_NAME="xatom-linx"',
             '-DBOARD_MANUFACTURER="Linx"',
             str(src2_path), cjson_obj, "-o", cls.binary2],
            check=True, capture_output=True, text=True)

    def test_helper_and_codec_binary(self):
        result = subprocess.run([self.binary], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_board_surface_binary(self):
        result = subprocess.run([self.binary2], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        out = {}
        for line in result.stdout.splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                out[k] = v
        minimal = json.loads(out["MINIMAL"])
        self.assertEqual(minimal["type"], "xatom")
        self.assertIs(minimal["network"]["ready"], False)
        json.loads(out["BOARD"])  # must be valid JSON too
        inv = json.loads(out["STATEINV"])["hardware"]
        self.assertFalse(inv["power"]["valid"])
        for k in ("battery_present", "battery_percent", "temperature_c", "battery_mv"):
            self.assertIsNone(inv["power"][k], k)
        for k in ("x", "y", "z"):
            self.assertIsNone(inv["motion"]["accel_mg"][k])
        self.assertIsNone(inv["radar"]["state"])
        self.assertIsNone(inv["radar"]["version"])
        mix = json.loads(out["STATEMIX"])["hardware"]
        self.assertTrue(mix["power"]["battery_present"])
        self.assertEqual(mix["power"]["battery_mv"], 4178)
        self.assertEqual(mix["power"]["temperature_c"], 22.0)
        # version-only radar: telemetry stays null, version published
        self.assertIsNone(mix["radar"]["state"])
        self.assertEqual(mix["radar"]["version"], "TEST")
        # failed sample: stale cached values publish null + exact SDK code
        err = json.loads(out["STATEERR"])["hardware"]["power"]
        self.assertFalse(err["valid"])
        self.assertEqual(err["last_error"], -4)  # ESP_ERR_INVALID_STATE
        self.assertIsNone(err["battery_mv"])  # was 4178 in cache
        self.assertIsNone(err["battery_percent"])

    def test_board_package_shape(self):
        cfg = json.loads((XATOM_DIR / "config.json").read_text())
        self.assertEqual(cfg["type"], "xatom")
        self.assertEqual(cfg["target"], "esp32s3")
        names = [b["name"] for b in cfg["builds"]]
        self.assertIn("xatom-linx", names)
        board = (XATOM_DIR / "xatom_board.cc").read_text()
        self.assertEqual(board.count("DECLARE_BOARD("), 1)
        self.assertNotIn("new lvgl", board.lower())
        # all timers must precede peripheral worker startup
        self.assertLess(board.index("power_save_timer_ = new"),
                        board.index("peripherals_.Start()"))
        self.assertLess(board.index("SetWakeCallback"),
                        board.index("peripherals_.Start()"))
        kconfig = (ROOT / "main/Kconfig.projbuild").read_text()
        self.assertIn("config BOARD_TYPE_XATOM", kconfig)
        cmake = (ROOT / "main/CMakeLists.txt").read_text()
        self.assertIn('set(BOARD_DIR "xatom")', cmake)
        # The variant must disable hotspot provisioning explicitly: hotspot
        # defaults y upstream and its #ifdef wins over the blufi #elif.
        app = cfg["builds"][names.index("xatom-linx")]["sdkconfig_append"]
        self.assertIn("CONFIG_USE_HOTSPOT_WIFI_PROVISIONING=n", app)
        self.assertIn("CONFIG_USE_ESP_BLUFI_WIFI_PROVISIONING=y", app)

    def test_start_wifi_config_mode_uses_blufi(self):
        """Actual WifiBoard::StartWifiConfigMode under the variant's macros:
        HOTSPOT undefined, BLUFI defined -> exactly one Blufi.init, zero AP."""
        wifi_board = (ROOT / "main/boards/common/wifi_board.cc").read_text()
        body = extract_function(wifi_board, "void WifiBoard::StartWifiConfigMode()")
        fixture = r'''
#include <cassert>
#include <cstdio>
#include <string_view>
static int g_blufi_init = 0;
static int g_start_ap = 0;
class Blufi { public: static Blufi& GetInstance() { static Blufi b; return b; }
    void init() { g_blufi_init++; } };
class WifiManager { public: static WifiManager& GetInstance() {
    static WifiManager w; return w; } void StartConfigAp() { g_start_ap++; }
    const char* GetApSsid() { return ""; } const char* GetApWebUrl() { return ""; } };
static constexpr int kDeviceStateWifiConfiguring = 1;
struct Lang { struct Strings { static constexpr const char* CONNECT_TO_HOTSPOT = "";
    static constexpr const char* ACCESS_VIA_BROWSER = "";
    static constexpr const char* ENTERING_WIFI_CONFIG_MODE = "";
    static constexpr const char* WIFI_CONFIG_MODE = ""; };
    struct Sounds { static constexpr const char* OGG_WIFICONFIG = ""; }; };
class Application { public: static Application& GetInstance() {
    static Application a; return a; } void SetDeviceState(int) {}
    template<typename F> void Schedule(F) {}
    void Alert(const char*, const char*, const char*, const std::string_view&) {} };
class WifiBoard { public: bool in_config_mode_ = false;
    void StartWifiConfigMode(); };
@@BODY@@
int main() {
    WifiBoard b;
    b.StartWifiConfigMode();
    assert(b.in_config_mode_);
    assert(g_blufi_init == 1);
    assert(g_start_ap == 0);
    printf("blufi-mode-ok\n");
    return 0;
}
'''
        src = fixture.replace("@@BODY@@", body)
        tmp = pathlib.Path(self.tmp.name)
        p = tmp / "wifi_cfg.cc"
        p.write_text(src)
        subprocess.run(
            ["c++", "-std=c++17", "-Wall", "-Werror",
             "-DCONFIG_USE_ESP_BLUFI_WIFI_PROVISIONING=1",
             # HOTSPOT deliberately NOT defined (#ifdef semantics); sdkconfig
             # emits value 1 for enabled features (real #elif evaluation)
             str(p), "-o", str(tmp / "wifi_cfg")],
            check=True, capture_output=True, text=True)
        r = subprocess.run([str(tmp / "wifi_cfg")], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("blufi-mode-ok", r.stdout)


if __name__ == "__main__":
    unittest.main()
