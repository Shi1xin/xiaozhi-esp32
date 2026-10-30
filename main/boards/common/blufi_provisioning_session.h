#pragma once

/*
 * Host-testable BluFi provisioning session state and pure helpers.
 *
 * Standard C++ only (mutex, optional, cstdint, string, algorithm) so the
 * production logic can be compiled and exercised off-target. The session
 * serializes the connect/report/deinit lifecycle behind one mutex; tokens
 * (attempt generations) let late callbacks from superseded workers fail
 * safely instead of mutating newer state. Never hold the mutex across SDK
 * calls, WifiManager calls, delays, or Bluetooth sends.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>

class BlufiProvisioningSession {
public:
    using Attempt = uint32_t;

    enum class DeinitResult { Proceed, Deferred, AlreadyStarted };

    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
        connecting_ = false;
        report_pending_ = false;
        provisioned_ = false;
        closing_ = false;
        ++generation_;
    }

    void SetBleConnected(bool connected) {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = connected;
        if (!connected && !report_pending_) {
            // Without a pending report the in-flight attempt cannot finish;
            // invalidate its token so late callbacks become no-ops.
            connecting_ = false;
            ++generation_;
        }
        // A pending report keeps its generation/window until its worker ends.
    }

    bool IsBleConnected() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

    bool IsConnecting() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return connecting_;
    }

    bool IsClosing() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closing_;
    }

    std::optional<Attempt> BeginConnection() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closing_ || !connected_) {
            return std::nullopt;
        }
        ++generation_;  // a retry invalidates the prior attempt atomically
        connecting_ = true;
        report_pending_ = false;
        provisioned_ = false;
        return generation_;
    }

    bool IsCurrent(Attempt attempt) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return attempt == generation_ && !closing_ && (connecting_ || report_pending_);
    }

    bool BeginReport(Attempt attempt) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (attempt != generation_ || !connected_ || !connecting_ || closing_) {
            return false;
        }
        // Single lock: no gap between dropping "connecting" and claiming
        // "report pending" where deinit could observe an idle session, and
        // provisioning success is marked with the same transition so a stale
        // worker can never paint it onto a newer attempt.
        report_pending_ = true;
        connecting_ = false;
        provisioned_ = true;
        return true;
    }

    bool IsProvisioned() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return provisioned_;
    }

    bool FinishReport(Attempt attempt) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (attempt != generation_ || !report_pending_) {
            return false;  // stale worker does nothing
        }
        report_pending_ = false;
        return true;
    }

    bool Cancel(Attempt attempt) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (attempt != generation_ || closing_ || (!connecting_ && !report_pending_)) {
            return false;  // only the matching current attempt may cancel
        }
        connecting_ = false;
        report_pending_ = false;
        provisioned_ = false;
        return true;
    }

    DeinitResult TryBeginDeinit() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closing_) {
            return DeinitResult::AlreadyStarted;
        }
        if (report_pending_ || (connected_ && connecting_)) {
            return DeinitResult::Deferred;
        }
        closing_ = true;  // future BeginConnection rejects the claimed close
        return DeinitResult::Proceed;
    }

private:
    mutable std::mutex mutex_;
    bool connected_ = false;
    bool connecting_ = false;
    bool report_pending_ = false;
    bool provisioned_ = false;
    bool closing_ = false;
    Attempt generation_ = 0;
};

namespace blufi_detail {

// WiFi mode values mirror wifi_mode_t without pulling in esp_wifi headers,
// keeping this file host-compilable.
constexpr int kWifiModeNull = 0;
constexpr int kWifiModeSta = 1;
constexpr int kWifiModeAp = 2;
constexpr int kWifiModeApsta = 3;

// Target mode for a provisioning scan: NULL bootstraps STA, AP preserves the
// hotspot by switching to APSTA, STA/APSTA are already scannable.
// Returns -1 for modes that cannot scan.
inline int BlufiScanTargetMode(int current_mode) {
    switch (current_mode) {
        case kWifiModeNull:
            return kWifiModeSta;
        case kWifiModeAp:
            return kWifiModeApsta;
        case kWifiModeSta:
        case kWifiModeApsta:
            return current_mode;
        default:
            return -1;
    }
}

constexpr size_t kMaxCachedAps = 64;
constexpr size_t kMaxSsidLen = 32;
constexpr size_t kMaxPasswordLen = 64;

inline size_t BlufiClampApCount(size_t count) { return std::min(count, kMaxCachedAps); }

// SSID must be 1..32 bytes; password 0..64 bytes (open networks legal).
inline bool BlufiValidSsidLen(size_t len) { return len >= 1 && len <= kMaxSsidLen; }

inline bool BlufiValidPasswordLen(size_t len) { return len <= kMaxPasswordLen; }

// Frame validation: SSID requires 1..32 bytes with a non-null payload;
// password allows len==0 (open network, NULL payload legal) up to 64 bytes
// and requires a non-null payload otherwise.
inline bool BlufiValidSsidFrame(int len, const uint8_t* src) {
    return len >= 1 && static_cast<size_t>(len) <= kMaxSsidLen && src != nullptr;
}

inline bool BlufiValidPasswordFrame(int len, const uint8_t* src) {
    return len >= 0 && static_cast<size_t>(len) <= kMaxPasswordLen && (len == 0 || src != nullptr);
}

// Clear the destination, then copy only the validated length. Never writes a
// terminator beyond the field — a full-length SSID/PSK is legal and callers
// construct std::string(ptr, len) when they need a terminated view. A zero
// length clears the destination without reading the source.
inline bool BlufiCopyCredential(uint8_t* dst, size_t dst_size, const uint8_t* src, size_t len) {
    if (dst == nullptr || len > dst_size || (len > 0 && src == nullptr)) {
        return false;
    }
    std::memset(dst, 0, dst_size);
    if (len > 0) {
        std::memcpy(dst, src, len);
    }
    return true;
}

// SCAN_DONE status: 0 means the scan completed, nonzero means it failed.
inline bool BlufiScanDoneOk(uint32_t status) { return status == 0; }

// A scan-failure response goes out only when a GET_WIFI_LIST request is
// deferred on this scan, the BLE link is still up, and no close was claimed.
inline bool BlufiShouldReportScanError(bool request_pending, bool ble_connected, bool closing) {
    return request_pending && ble_connected && !closing;
}

// Advertising config tracker: start only after BOTH data-set completions
// succeed; a non-success completion abandons the pending advertisement.
struct BlufiAdvTracker {
    bool adv_done = false;
    bool rsp_done = false;
    bool pending = false;
};

enum class BlufiAdvAction { Wait, Start, Abandon };

inline BlufiAdvAction BlufiOnAdvConfigComplete(BlufiAdvTracker& tracker, bool adv_data,
                                               bool success) {
    if (adv_data) {
        tracker.adv_done = success;
    } else {
        tracker.rsp_done = success;
    }
    if (!success) {
        tracker.pending = false;
        return BlufiAdvAction::Abandon;
    }
    if (tracker.pending && tracker.adv_done && tracker.rsp_done) {
        tracker.pending = false;
        return BlufiAdvAction::Start;
    }
    return BlufiAdvAction::Wait;
}

// GET_WIFI_LIST dispatch: defer to an in-flight scan, answer from the cache,
// or start a fresh scan that will answer from its done handler.
enum class BlufiWifiListAction { DeferToInFlight, RespondCached, StartFreshScan };

inline BlufiWifiListAction BlufiResolveWifiListAction(bool scan_in_flight, bool cache_non_empty) {
    if (scan_in_flight) {
        return BlufiWifiListAction::DeferToInFlight;
    }
    if (cache_non_empty) {
        return BlufiWifiListAction::RespondCached;
    }
    return BlufiWifiListAction::StartFreshScan;
}

// Optional MAC-bearing discovery name: ESP_<12 lowercase hex STA MAC>.
inline std::string BlufiFormatDeviceName(const uint8_t mac[6]) {
    char name[17];
    std::snprintf(name, sizeof(name), "ESP_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2],
                  mac[3], mac[4], mac[5]);
    return std::string(name);
}

}  // namespace blufi_detail
