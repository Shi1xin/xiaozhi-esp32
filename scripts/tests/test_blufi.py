"""Host-compile and exercise the real BluFi provisioning production code."""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
COMMON = ROOT / "main/boards/common"
SESSION_HEADER = COMMON / "blufi_provisioning_session.h"
BLUFI_CPP = COMMON / "blufi.cpp"


def extract_function(source, signature):
    """Extract a complete function definition from blufi.cpp by brace counting."""
    start = source.index(signature)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


# ---------------------------------------------------------------------------
# Production method extraction: the fixture below compiles the actual method
# bodies from blufi.cpp against stubbed SDK entry points, so tests exercise
# the real code (not a reimplementation).
# ---------------------------------------------------------------------------
PROD_METHODS = [
    "bool Blufi::start_wifi_scan()",
    "void Blufi::_send_wifi_list()",
    "void Blufi::_wifi_scan_event_handler",
    "esp_err_t Blufi::_ensure_scan_handler_registered()",
    "void Blufi::_handle_get_wifi_list()",
    "void Blufi::_takeover_radio_for_connect()",
    "void Blufi::_release_scan_for_close()",
]

FIXTURE_SOURCE = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "main/boards/common/blufi_provisioning_session.h"

// ---- SDK stubs ---------------------------------------------------------
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr esp_err_t ESP_ERR_WIFI_STATE = 0x300b;
const char* esp_err_to_name(esp_err_t) { return "stub_err"; }
#define ESP_LOGI(tag, ...)
#define ESP_LOGW(tag, ...)
#define ESP_LOGE(tag, ...)

enum wifi_mode_t { WIFI_MODE_NULL = 0, WIFI_MODE_STA = 1, WIFI_MODE_AP = 2, WIFI_MODE_APSTA = 3 };
struct wifi_ap_record_t {
    uint8_t ssid[33];
    int8_t rssi;
    int authmode;
    uint8_t bssid[6];
};
struct wifi_event_sta_scan_done_t {
    uint32_t status;
    uint32_t number;
    uint8_t scan_id;
};
using esp_event_base_t = const char*;
using esp_event_handler_instance_t = void*;
constexpr esp_event_base_t WIFI_EVENT = "WIFI_EVENT";
constexpr int32_t WIFI_EVENT_SCAN_DONE = 1;
using esp_event_handler_t = void (*)(void*, esp_event_base_t, int32_t, void*);

struct esp_blufi_ap_record_t {
    uint8_t ssid[33];
    int8_t rssi;
};
enum esp_blufi_error_state_t { ESP_BLUFI_WIFI_SCAN_FAIL = 10 };

// ---- call counters / controllable SDK behaviour -------------------------
struct StubCounts {
    int register_handler = 0;
    int get_mode = 0;
    int set_mode = 0;
    int wifi_start = 0;
    int scan_start = 0;
    int scan_stop = 0;
    int get_ap_num = 0;
    int get_ap_records = 0;
    int send_wifi_list = 0;
    int send_error = 0;
    int wifi_list_records = 0;
};
static StubCounts C;
static esp_err_t g_get_mode_rc = ESP_OK;
static wifi_mode_t g_mode = WIFI_MODE_STA;
static esp_err_t g_set_mode_rc = ESP_OK;
static esp_err_t g_wifi_start_rc = ESP_OK;
static esp_err_t g_scan_start_rc = ESP_OK;
static esp_err_t g_get_num_rc = ESP_OK;
static esp_err_t g_get_records_rc = ESP_OK;
static esp_err_t g_register_rc = ESP_OK;
static uint16_t g_num_aps = 0;
static void (*g_during_get_num)() = nullptr;
static void (*g_during_get_mode)() = nullptr;
static void reset_stubs() {
    C = StubCounts{};
    g_get_mode_rc = ESP_OK; g_mode = WIFI_MODE_STA;
    g_set_mode_rc = ESP_OK; g_wifi_start_rc = ESP_OK;
    g_scan_start_rc = ESP_OK; g_get_num_rc = ESP_OK;
    g_get_records_rc = ESP_OK; g_register_rc = ESP_OK;
    g_num_aps = 0; g_during_get_num = nullptr; g_during_get_mode = nullptr;
}

esp_err_t esp_event_handler_instance_register(esp_event_base_t base, int32_t id,
                                              esp_event_handler_t, void*,
                                              esp_event_handler_instance_t* inst) {
    C.register_handler++;
    if (g_register_rc == ESP_OK) *inst = (void*)0x1;
    return g_register_rc;
}
esp_err_t esp_wifi_get_mode(wifi_mode_t* m) {
    C.get_mode++;
    *m = g_mode;
    if (g_during_get_mode) { auto f = g_during_get_mode; g_during_get_mode = nullptr; f(); }
    return g_get_mode_rc;
}
esp_err_t esp_wifi_set_mode(wifi_mode_t m) { C.set_mode++; g_mode = m; return g_set_mode_rc; }
esp_err_t esp_wifi_start() { C.wifi_start++; return g_wifi_start_rc; }
esp_err_t esp_wifi_scan_start(void*, bool) { C.scan_start++; return g_scan_start_rc; }
esp_err_t esp_wifi_scan_stop() { C.scan_stop++; return ESP_OK; }
esp_err_t esp_wifi_scan_get_ap_num(uint16_t* n) {
    C.get_ap_num++;
    *n = g_num_aps;
    if (g_during_get_num) { auto f = g_during_get_num; g_during_get_num = nullptr; f(); }
    return g_get_num_rc;
}
esp_err_t esp_wifi_scan_get_ap_records(uint16_t* n, wifi_ap_record_t* recs) {
    C.get_ap_records++;
    uint16_t k = std::min<uint16_t>(*n, g_num_aps);
    for (uint16_t i = 0; i < k; ++i) {
        recs[i] = {};
        memcpy(recs[i].ssid, "stub_ap", 8);
        recs[i].rssi = -50;
    }
    *n = k;
    return g_get_records_rc;
}
void esp_blufi_send_error_info(int) { C.send_error++; }
void esp_blufi_send_wifi_list(uint16_t num, esp_blufi_ap_record_t*) {
    C.send_wifi_list++;
    C.wifi_list_records = num;
}

// ---- fixture class with production member fields ------------------------
class Blufi {
public:
    BlufiProvisioningSession m_session;
    std::mutex m_scan_mutex;
    std::vector<wifi_ap_record_t> m_ap_records;
    bool m_scan_in_progress = false;
    uint32_t m_scan_generation = 0;
    esp_event_handler_instance_t m_scan_event_instance = nullptr;
    bool m_scan_should_save_ssid = true;
    bool m_send_list_after_scan = false;

    bool start_wifi_scan();
    void _send_wifi_list();
    static void _wifi_scan_event_handler(void*, esp_event_base_t, int32_t, void*);
    esp_err_t _ensure_scan_handler_registered();
    void _handle_get_wifi_list();
    void _takeover_radio_for_connect();
    void _release_scan_for_close();
};

@@PROD_METHODS@@

// ---- production-path tests ---------------------------------------------
static void test_register_once() {
    reset_stubs();
    Blufi b;
    assert(b._ensure_scan_handler_registered() == ESP_OK);
    assert(b._ensure_scan_handler_registered() == ESP_OK);
    assert(C.register_handler == 1);
    // registration failure propagates and instance stays null
    reset_stubs();
    Blufi b2;
    g_register_rc = ESP_FAIL;
    assert(b2._ensure_scan_handler_registered() == ESP_FAIL);
    assert(b2.m_scan_event_instance == nullptr);
}

static void test_scan_done_success() {
    reset_stubs();
    Blufi b;
    b.m_session.SetBleConnected(true);
    b.m_scan_in_progress = true;
    b.m_scan_should_save_ssid = true;
    b.m_send_list_after_scan = true;
    b.m_scan_generation = 7;
    g_num_aps = 3;
    wifi_event_sta_scan_done_t ev{0, 3, 0};
    Blufi::_wifi_scan_event_handler(&b, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ev);
    assert(C.get_ap_num == 1 && C.get_ap_records == 1);
    assert(C.send_wifi_list == 1 && C.wifi_list_records == 3);
    // The send consumes the cache and kicks off an async refresh scan.
    assert(b.m_ap_records.empty() && !b.m_send_list_after_scan);
    assert(C.scan_start == 1);
    assert(C.send_error == 0);
}

static void test_scan_done_failure_status() {
    reset_stubs();
    Blufi b;
    b.m_session.SetBleConnected(true);
    b.m_scan_in_progress = true;
    b.m_send_list_after_scan = true;
    wifi_event_sta_scan_done_t ev{1, 0, 0};
    Blufi::_wifi_scan_event_handler(&b, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ev);
    assert(C.get_ap_num == 0 && C.get_ap_records == 0);
    assert(b.m_ap_records.empty());          // failed scan never caches
    assert(!b.m_scan_in_progress);
    assert(C.send_error == 1 && C.send_wifi_list == 0);
    // null event_data behaves identically
    reset_stubs();
    Blufi b2;
    b2.m_session.SetBleConnected(true);
    b2.m_scan_in_progress = true;
    b2.m_send_list_after_scan = true;
    Blufi::_wifi_scan_event_handler(&b2, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, nullptr);
    assert(C.get_ap_num == 0 && C.send_error == 1);
    // failure with no pending request sends nothing
    reset_stubs();
    Blufi b3;
    b3.m_session.SetBleConnected(true);
    b3.m_scan_in_progress = true;
    Blufi::_wifi_scan_event_handler(&b3, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, nullptr);
    assert(C.send_error == 0);
}

static void test_station_owned_scan_ignored() {
    reset_stubs();
    Blufi b;
    b.m_session.SetBleConnected(true);
    wifi_event_sta_scan_done_t ev{0, 9, 0};
    g_num_aps = 9;
    Blufi::_wifi_scan_event_handler(&b, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ev);
    assert(C.get_ap_num == 0 && C.get_ap_records == 0);
    assert(b.m_ap_records.empty() && C.send_wifi_list == 0 && C.send_error == 0);
}

static Blufi* g_revoke_target = nullptr;
static void revoke_and_rescan() {
    g_revoke_target->_takeover_radio_for_connect();
    g_scan_start_rc = ESP_OK;
    g_get_mode_rc = ESP_OK; g_mode = WIFI_MODE_STA;
    g_revoke_target->start_wifi_scan();  // newer scan begins immediately
}

static void test_revoked_callback_noop() {
    reset_stubs();
    Blufi b;
    g_revoke_target = &b;
    b.m_session.SetBleConnected(true);
    b.m_scan_in_progress = true;
    b.m_scan_should_save_ssid = true;
    b.m_send_list_after_scan = true;         // pending released once by takeover
    b.m_scan_generation = 5;
    g_num_aps = 4;
    g_during_get_num = revoke_and_rescan;  // revoke + rescan mid-fetch
    wifi_event_sta_scan_done_t ev{0, 4, 0};
    Blufi::_wifi_scan_event_handler(&b, WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &ev);
    // stale callback fetched count but never records, never published cache,
    // and never cleared the newer in-flight scan.
    assert(C.get_ap_num == 1 && C.get_ap_records == 0);
    assert(b.m_ap_records.empty());
    assert(b.m_scan_in_progress);            // newer scan survives
    assert(C.send_error == 1);               // takeover released pending once
    assert(C.send_wifi_list == 0);
    g_revoke_target = nullptr;
}

static Blufi* g_revoke_mode_target = nullptr;
static void revoke_and_rescan_at_get_mode() {
    auto* b = g_revoke_mode_target;
    b->_takeover_radio_for_connect();      // clears flags, bumps generation
    // A newer dedicated scan + deferred request begin immediately.
    g_scan_start_rc = ESP_OK;
    b->start_wifi_scan();
    {
        std::lock_guard<std::mutex> lock(b->m_scan_mutex);
        b->m_send_list_after_scan = true;
    }
    g_scan_start_rc = ESP_FAIL;            // the older claim now fails
}

static void test_failed_older_start_keeps_newer_scan() {
    reset_stubs();
    Blufi b;
    g_revoke_mode_target = &b;
    b.m_session.SetBleConnected(true);
    b.m_send_list_after_scan = true;
    g_during_get_mode = revoke_and_rescan_at_get_mode;
    // The older claim reaches scan_start after the takeover+rescan; its
    // failure must not clear the newer scan's ownership or pending request.
    assert(!b.start_wifi_scan());
    assert(b.m_scan_in_progress);            // newer scan still owns
    assert(b.m_send_list_after_scan);        // newer pending request intact
    assert(C.send_error == 1);               // exactly the takeover's release
    assert(C.scan_start == 1);               // only the newer scan hit the SDK
    g_revoke_mode_target = nullptr;
}

static void test_get_list_cached_while_connecting() {
    reset_stubs();
    Blufi b;
    b.m_session.SetBleConnected(true);
    auto a = b.m_session.BeginConnection();
    assert(a.has_value());
    wifi_ap_record_t rec{};
    memcpy(rec.ssid, "cached", 7);
    b.m_ap_records.push_back(rec);
    b._handle_get_wifi_list();
    assert(C.send_wifi_list == 1 && C.wifi_list_records == 1);
    assert(C.scan_start == 0);               // no dedicated scan while connecting
    // cache cleared by the send; second request emits one error, still no scan
    b._handle_get_wifi_list();
    assert(C.send_wifi_list == 1 && C.send_error == 1 && C.scan_start == 0);
}

static void test_scan_start_failures() {
    // get_mode failure: exactly one error for the deferred request.
    reset_stubs();
    Blufi b;
    b.m_session.SetBleConnected(true);
    b.m_send_list_after_scan = true;
    g_get_mode_rc = ESP_FAIL;
    assert(!b.start_wifi_scan());
    assert(C.send_error == 1 && !b.m_send_list_after_scan && !b.m_scan_in_progress);
    // wifi_start failure: same single-error release.
    reset_stubs();
    Blufi b2;
    b2.m_session.SetBleConnected(true);
    b2.m_send_list_after_scan = true;
    g_wifi_start_rc = ESP_FAIL;
    assert(!b2.start_wifi_scan());
    assert(C.send_error == 1);
    // scan_start failure: same.
    reset_stubs();
    Blufi b3;
    b3.m_session.SetBleConnected(true);
    b3.m_send_list_after_scan = true;
    g_scan_start_rc = ESP_FAIL;
    assert(!b3.start_wifi_scan());
    assert(C.send_error == 1);
    // disconnected: pending released silently.
    reset_stubs();
    Blufi b4;
    b4.m_send_list_after_scan = true;
    g_get_mode_rc = ESP_FAIL;
    assert(!b4.start_wifi_scan());
    assert(C.send_error == 0);
    // closing: no frames at all.
    reset_stubs();
    Blufi b5;
    b5.m_session.SetBleConnected(true);
    b5.m_session.TryBeginDeinit();
    b5.m_send_list_after_scan = true;
    g_get_mode_rc = ESP_FAIL;
    assert(!b5.start_wifi_scan());
    assert(C.send_error == 0 && C.get_mode == 0);
}

static void test_start_scan_mode_bootstrap() {
    // NULL -> STA bootstrap, AP -> APSTA preserves hotspot.
    for (auto [from, want] : std::initializer_list<std::pair<wifi_mode_t, wifi_mode_t>>{
             {WIFI_MODE_NULL, WIFI_MODE_STA}, {WIFI_MODE_AP, WIFI_MODE_APSTA},
             {WIFI_MODE_STA, WIFI_MODE_STA}, {WIFI_MODE_APSTA, WIFI_MODE_APSTA}}) {
        reset_stubs();
        Blufi b;
        b.m_session.SetBleConnected(true);
        g_mode = from;
        assert(b.start_wifi_scan());
        assert(g_mode == want);
        assert(C.scan_start == 1 && b.m_scan_in_progress);
    }
    // connecting/closing never calls Wi-Fi APIs.
    reset_stubs();
    Blufi b2;
    b2.m_session.SetBleConnected(true);
    b2.m_session.BeginConnection();
    assert(!b2.start_wifi_scan());
    assert(C.get_mode == 0 && C.wifi_start == 0 && C.scan_start == 0);
}

int main() {
    test_register_once();
    test_scan_done_success();
    test_scan_done_failure_status();
    test_station_owned_scan_ignored();
    test_revoked_callback_noop();
    test_failed_older_start_keeps_newer_scan();
    test_get_list_cached_while_connecting();
    test_scan_start_failures();
    test_start_scan_mode_bootstrap();
    return 0;
}
'''

HELPER_SOURCE = r'''
#include <cassert>
#include <cstring>
#include <string>
#include "main/boards/common/blufi_provisioning_session.h"

using namespace blufi_detail;
using Attempt = BlufiProvisioningSession::Attempt;
using DeinitResult = BlufiProvisioningSession::DeinitResult;

static void test_session_lifecycle() {
    BlufiProvisioningSession s;
    s.Reset();
    assert(s.TryBeginDeinit() == DeinitResult::Proceed);
    assert(s.TryBeginDeinit() == DeinitResult::AlreadyStarted);
    s.Reset();
    assert(!s.IsClosing());

    s.SetBleConnected(true);
    auto a = s.BeginConnection();
    assert(a.has_value());
    assert(s.TryBeginDeinit() == DeinitResult::Deferred);
    assert(s.BeginReport(*a));
    assert(s.TryBeginDeinit() == DeinitResult::Deferred);
    assert(s.FinishReport(*a));
    assert(s.TryBeginDeinit() == DeinitResult::Proceed);

    s.Reset();
    s.SetBleConnected(true);
    auto b = s.BeginConnection();
    assert(b.has_value());
    s.SetBleConnected(false);
    assert(!s.IsCurrent(*b));
    assert(!s.BeginReport(*b));
    assert(!s.FinishReport(*b));

    s.Reset();
    s.SetBleConnected(true);
    auto c = s.BeginConnection();
    assert(c.has_value());
    assert(s.BeginReport(*c));
    s.SetBleConnected(false);
    assert(s.TryBeginDeinit() == DeinitResult::Deferred);
    assert(s.FinishReport(*c));
    assert(s.TryBeginDeinit() == DeinitResult::Proceed);

    s.Reset();
    s.SetBleConnected(true);
    auto d1 = s.BeginConnection();
    auto d2 = s.BeginConnection();
    assert(d1.has_value() && d2.has_value() && *d1 != *d2);
    assert(!s.BeginReport(*d1));
    assert(!s.FinishReport(*d1));
    assert(!s.Cancel(*d1));
    assert(s.IsCurrent(*d2));

    s.Reset();
    s.SetBleConnected(true);
    auto e1 = s.BeginConnection();
    assert(e1.has_value());
    assert(s.Cancel(*e1));
    assert(s.TryBeginDeinit() == DeinitResult::Proceed);
    s.Reset();
    s.SetBleConnected(true);
    auto e2 = s.BeginConnection();
    assert(e2.has_value() && *e2 != *e1);
    assert(!s.Cancel(*e1));
    assert(s.IsCurrent(*e2));

    s.Reset();
    s.SetBleConnected(true);
    assert(s.TryBeginDeinit() == DeinitResult::Proceed);
    assert(!s.BeginConnection().has_value());
    s.Reset();
    assert(!s.BeginConnection().has_value());
}

static void test_provisioned_ownership() {
    BlufiProvisioningSession s;
    s.Reset();
    s.SetBleConnected(true);
    assert(!s.IsProvisioned());
    auto p1 = s.BeginConnection();
    assert(p1.has_value() && !s.IsProvisioned());
    assert(s.BeginReport(*p1));
    assert(s.IsProvisioned());
    auto p2 = s.BeginConnection();
    assert(p2.has_value());
    assert(!s.IsProvisioned());
    assert(!s.BeginReport(*p1));
    assert(!s.IsProvisioned());
    assert(s.BeginReport(*p2));
    assert(s.IsProvisioned());
    assert(!s.Cancel(*p1));
    assert(s.IsProvisioned());
    assert(s.Cancel(*p2));
    assert(!s.IsProvisioned());
}

static void test_credential_boundaries() {
    struct Guarded32 { uint8_t pre; uint8_t buf[32]; uint8_t post; };
    struct Guarded64 { uint8_t pre; uint8_t buf[64]; uint8_t post; };
    Guarded32 g32{0xCC, {}, 0xDD};
    Guarded64 g64{0xEE, {}, 0xFF};
    std::memset(g32.buf, 0xAA, sizeof(g32.buf));
    std::memset(g64.buf, 0xAA, sizeof(g64.buf));

    uint8_t ssid32[32];
    std::memset(ssid32, 'x', sizeof(ssid32));
    uint8_t psk[64];
    std::memset(psk, 'k', sizeof(psk));

    assert(BlufiValidSsidFrame(32, ssid32));
    assert(!BlufiValidSsidFrame(0, ssid32));
    assert(!BlufiValidSsidFrame(33, ssid32));
    assert(!BlufiValidSsidFrame(5, nullptr));
    assert(BlufiValidPasswordFrame(64, psk));
    assert(BlufiValidPasswordFrame(0, nullptr));
    assert(!BlufiValidPasswordFrame(65, psk));
    assert(!BlufiValidPasswordFrame(-1, psk));
    assert(!BlufiValidPasswordFrame(3, nullptr));

    assert(BlufiCopyCredential(g32.buf, sizeof(g32.buf), ssid32, 32));
    assert(BlufiCopyCredential(g64.buf, sizeof(g64.buf), psk, 64));
    assert(!BlufiCopyCredential(g32.buf, sizeof(g32.buf), ssid32, 33));
    assert(g32.pre == 0xCC && g32.post == 0xDD);
    assert(g64.pre == 0xEE && g64.post == 0xFF);

    assert(BlufiCopyCredential(g64.buf, sizeof(g64.buf), nullptr, 0));
    assert(g64.buf[0] == 0 && g64.buf[63] == 0);
    assert(g64.pre == 0xEE && g64.post == 0xFF);

    std::memset(g32.buf, 0xAA, sizeof(g32.buf));
    assert(BlufiCopyCredential(g32.buf, sizeof(g32.buf), ssid32, 4));
    assert(std::memcmp(g32.buf, ssid32, 4) == 0);
    assert(g32.buf[4] == 0 && g32.buf[31] == 0);
    assert(g32.pre == 0xCC && g32.post == 0xDD);

    assert(BlufiValidSsidLen(1) && BlufiValidSsidLen(32) && !BlufiValidSsidLen(0));
    assert(!BlufiValidSsidLen(33));
    assert(BlufiValidPasswordLen(0) && BlufiValidPasswordLen(64));
    assert(!BlufiValidPasswordLen(65));
    int invalid_len = -1;
    assert(!BlufiValidPasswordLen(static_cast<size_t>(invalid_len)));
}

static void test_scan_decisions() {
    assert(BlufiScanTargetMode(kWifiModeNull) == kWifiModeSta);
    assert(BlufiScanTargetMode(kWifiModeAp) == kWifiModeApsta);
    assert(BlufiScanTargetMode(kWifiModeSta) == kWifiModeSta);
    assert(BlufiScanTargetMode(kWifiModeApsta) == kWifiModeApsta);
    assert(BlufiScanTargetMode(99) == -1);
    assert(BlufiScanDoneOk(0));
    assert(!BlufiScanDoneOk(1));
    assert(BlufiResolveWifiListAction(true, true) == BlufiWifiListAction::DeferToInFlight);
    assert(BlufiResolveWifiListAction(true, false) == BlufiWifiListAction::DeferToInFlight);
    assert(BlufiResolveWifiListAction(false, true) == BlufiWifiListAction::RespondCached);
    assert(BlufiResolveWifiListAction(false, false) == BlufiWifiListAction::StartFreshScan);
    assert(BlufiClampApCount(0) == 0);
    assert(BlufiClampApCount(64) == 64);
    assert(BlufiClampApCount(200) == 64);
    assert(BlufiShouldReportScanError(true, true, false));
    assert(!BlufiShouldReportScanError(false, true, false));
    assert(!BlufiShouldReportScanError(true, false, false));
    assert(!BlufiShouldReportScanError(true, true, true));
}

static void test_adv_tracker() {
    for (bool adv_first : {true, false}) {
        BlufiAdvTracker t;
        t.pending = true;
        auto first = BlufiOnAdvConfigComplete(t, adv_first, true);
        auto second = BlufiOnAdvConfigComplete(t, !adv_first, true);
        assert(first == BlufiAdvAction::Wait);
        assert(second == BlufiAdvAction::Start);
        assert(!t.pending);
        assert(BlufiOnAdvConfigComplete(t, true, true) == BlufiAdvAction::Wait);
    }
    BlufiAdvTracker t2;
    t2.pending = true;
    assert(BlufiOnAdvConfigComplete(t2, true, true) == BlufiAdvAction::Wait);
    assert(t2.pending);
    assert(BlufiOnAdvConfigComplete(t2, false, false) == BlufiAdvAction::Abandon);
    assert(!t2.pending);
    assert(BlufiOnAdvConfigComplete(t2, true, true) == BlufiAdvAction::Wait);
}

static void test_adv_payload_budget() {
    constexpr size_t kPrimaryAdvBytes = 18 + 3 + 3 + 6;
    static_assert(kPrimaryAdvBytes <= 31, "primary ADV overflows");
    const uint8_t mac[6] = {0x02, 0x12, 0x34, 0x56, 0x78, 0x9a};
    const std::string name = BlufiFormatDeviceName(mac);
    assert(name == "ESP_02123456789a");
    assert(name.size() + 2 <= 31);
    const uint8_t mac2[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    assert(BlufiFormatDeviceName(mac2) == "ESP_000000000000");
}

int main() {
    test_session_lifecycle();
    test_provisioned_ownership();
    test_credential_boundaries();
    test_scan_decisions();
    test_adv_tracker();
    test_adv_payload_budget();
    return 0;
}
'''


def build_fixture_source():
    cpp = BLUFI_CPP.read_text()
    bodies = "\n".join(extract_function(cpp, sig) for sig in PROD_METHODS)
    return FIXTURE_SOURCE.replace("@@PROD_METHODS@@", bodies)


@unittest.skipUnless(shutil.which("c++"), "requires a host C++ compiler")
class BlufiProvisioningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.binaries = {}
        for name, source in (("helpers", HELPER_SOURCE), ("prod", build_fixture_source())):
            src = pathlib.Path(cls.tmp.name) / f"{name}.cc"
            src.write_text(source)
            binary = str(pathlib.Path(cls.tmp.name) / f"blufi-{name}")
            subprocess.run(
                ["c++", "-std=c++17", "-Wall", "-Werror", "-I", str(ROOT),
                 str(src), "-o", binary],
                check=True, capture_output=True, text=True)
            cls.binaries[name] = binary

    def test_session_helpers_and_adv(self):
        result = subprocess.run([self.binaries["helpers"]], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_production_scan_methods(self):
        result = subprocess.run([self.binaries["prod"]], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_session_header_is_standard_only(self):
        for line in SESSION_HEADER.read_text().splitlines():
            if line.strip().startswith("#include"):
                for forbidden in ("esp_", "freertos", "driver/", "soc/"):
                    self.assertNotIn(forbidden, line)

    def test_req_connect_stops_radio_before_start_station(self):
        # WifiStation starts scanning on WIFI_EVENT_STA_START, which a
        # repeated esp_wifi_start() does not re-emit. The dedicated BluFi
        # scan can leave the driver running, so the connect request must
        # stop the radio before StartStation or the station never scans.
        cpp = BLUFI_CPP.read_text()
        case_start = cpp.index("ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP")
        case_end = cpp.index("ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP")
        case_body = cpp[case_start:case_end]
        stop_pos = case_body.index("esp_wifi_stop")
        start_pos = case_body.index("wifi_manager.StartStation()")
        self.assertLess(stop_pos, start_pos,
                        "esp_wifi_stop() must precede StartStation() so the "
                        "station restarts the driver and gets STA_START")


if __name__ == "__main__":
    unittest.main()
