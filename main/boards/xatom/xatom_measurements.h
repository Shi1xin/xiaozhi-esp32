#pragma once

/*
 * X-Atom measurement helpers — standard C++ only so the production
 * conversion and parsing code compiles and runs on the host. All formulas
 * and frame rules are fixed board constants; nothing here
 * derives units or invents calibration.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace xatom {

// ---- PMIC (AXP2101 at 0x35, chip ID 0x4A) -------------------------------

struct PmicSnapshot {
    bool available = false;  // device was detected at init
    bool valid = false;      // every required read of this sample succeeded
    int last_error = 0;
    int64_t sample_us = 0;

    bool battery_present = false;
    bool vbus_good = false;
    bool charging = false;
    bool discharging = false;
    bool charging_done = false;
    bool soc_valid = false;  // SOC within 0..100 AND battery present
    int battery_percent = 0;
    int battery_mv = 0;
    int vbus_mv = 0;
    bool temperature_valid = false;  // raw != 0 and within [-40, 125]
    float temperature_c = 0.0f;
};

// Convert a raw register set into the cached snapshot fields. Returns false
// when a required read failed — the caller publishes only on success.
inline bool PmicDecode(uint8_t status0, uint8_t status1, uint8_t bat_high, uint8_t bat_low,
                       uint8_t vbus_high, uint8_t vbus_low, uint8_t temp_high, uint8_t temp_low,
                       uint8_t soc, PmicSnapshot* out) {
    if (out == nullptr) {
        return false;
    }
    out->battery_present = (status0 & 0x08) != 0;
    out->vbus_good = (status0 & 0x20) != 0;
    const uint8_t current_direction = (status1 >> 5) & 0x03;
    out->charging = current_direction == 1;
    out->discharging = current_direction == 2;
    out->charging_done = (status1 & 0x07) == 0x04;
    out->battery_mv = ((static_cast<int>(bat_high) & 0x1f) << 8) | bat_low;
    out->vbus_mv = ((static_cast<int>(vbus_high) & 0x3f) << 8) | vbus_low;
    const int temperature_raw = ((static_cast<int>(temp_high) & 0x3f) << 8) | temp_low;
    // Authored conversion: 22.0 + (7274 - raw) / 20.0, valid only for a
    // nonzero raw reading inside [-40, 125] C.
    const float temperature_c = 22.0f + (7274.0f - temperature_raw) / 20.0f;
    out->temperature_valid =
        temperature_raw != 0 && temperature_c >= -40.0f && temperature_c <= 125.0f;
    out->temperature_c = out->temperature_valid ? temperature_c : 0.0f;
    out->soc_valid = out->battery_present && soc <= 100;
    out->battery_percent = out->soc_valid ? soc : 0;
    return true;
}

// GetBatteryLevel predicate: SOC must be 0..100 with a present battery.
inline bool PmicBatteryLevelValid(const PmicSnapshot& s) { return s.valid && s.soc_valid; }

// ---- QMI8658 IMU (WHO_AM_I 0x05 at 0x6B) --------------------------------

struct ImuSample {
    bool available = false;
    bool valid = false;
    int last_error = 0;
    int64_t sample_us = 0;
    float accel_mg[3] = {0, 0, 0};  // milli-g, raw * 1000 / 4096
    int gyro_raw[3] = {0, 0, 0};    // raw counts (CTRL3=0x66 preset, units TBD)
    int irq_level = 0;              // GPIO42 raw level
};

// Decode 12 bytes from reg 0x35: little-endian int16 AX/AY/AZ/GX/GY/GZ.
inline bool ImuDecode(const uint8_t* data, size_t len, ImuSample* out) {
    if (out == nullptr || data == nullptr || len < 12) {
        return false;
    }
    int16_t v[6];
    for (int i = 0; i < 6; i++) {
        v[i] = static_cast<int16_t>(static_cast<uint16_t>(data[i * 2]) |
                                    (static_cast<uint16_t>(data[i * 2 + 1]) << 8));
    }
    for (int i = 0; i < 3; i++) {
        out->accel_mg[i] = static_cast<float>(v[i]) * 1000.0f / 4096.0f;
        out->gyro_raw[i] = v[i + 3];
    }
    return true;
}

// ---- Radar (UART2, RKB1161 frames) ---------------------------------------

// Resynchronizing streaming parser. Frame layout:
//   55 A5 | len_hi len_lo (BE) | func(1) | cmd_hi cmd_lo | payload | checksum
// len counts func+cmd+payload+checksum (valid 4..252); total frame = len + 4
// (2-byte header + 2-byte length). checksum = 8-bit sum of all preceding
// frame bytes. Callers Append each received byte exactly once and then loop
// Next() until it returns false — never refeed bytes.
class XAtomRadarParser {
public:
    struct Frame {
        uint8_t func = 0;
        uint16_t cmd = 0;
        uint8_t payload[248] = {};
        size_t payload_len = 0;
    };

    // Store one received byte; the bounded buffer drops its oldest byte when
    // full so resync is always possible.
    void Append(uint8_t b) {
        if (pos_ < kMaxBuf) {
            buf_[pos_++] = b;
        } else {
            std::memmove(buf_, buf_ + 1, kMaxBuf - 1);
            buf_[kMaxBuf - 1] = b;
        }
    }

    // Extract the next complete valid frame into *f. Returns false when the
    // buffered data is incomplete, noise, or has been fully consumed.
    bool Next(Frame* f) {
        while (pos_ > 0) {
            // Align on the header, retaining a trailing 0x55.
            if (buf_[0] != 0x55) {
                size_t i = 0;
                while (i < pos_ && buf_[i] != 0x55)
                    i++;
                if (i == pos_) {
                    pos_ = 0;
                    return false;
                }
                std::memmove(buf_, buf_ + i, pos_ - i);
                pos_ -= i;
            }
            if (pos_ < 2) {
                return false;
            }
            if (buf_[1] != 0xA5) {
                Drop(1);
                continue;
            }
            if (pos_ < 4) {
                return false;
            }
            const uint16_t flen = (static_cast<uint16_t>(buf_[2]) << 8) | buf_[3];
            if (flen < 4 || flen > 252) {
                Drop(1);
                continue;
            }
            const size_t total = static_cast<size_t>(flen) + 4;
            if (pos_ < total) {
                return false;  // wait for more bytes
            }
            uint8_t sum = 0;
            for (size_t i = 0; i < total - 1; i++) {
                sum += buf_[i];
            }
            if (sum != buf_[total - 1]) {
                Drop(1);  // bad checksum: retry from the next byte
                continue;
            }
            f->func = buf_[4];
            f->cmd = static_cast<uint16_t>((buf_[5] << 8) | buf_[6]);
            f->payload_len = static_cast<size_t>(flen) - 4;
            if (f->payload_len > 0) {
                std::memcpy(f->payload, buf_ + 7, f->payload_len);
            }
            Drop(total);
            return true;
        }
        return false;
    }

    void Reset() { pos_ = 0; }

private:
    static constexpr size_t kMaxBuf = 256;
    uint8_t buf_[kMaxBuf] = {};
    size_t pos_ = 0;

    void Drop(size_t n) {
        if (n >= pos_) {
            pos_ = 0;
        } else {
            std::memmove(buf_, buf_ + n, pos_ - n);
            pos_ -= n;
        }
    }
};

// Version reply: func=2, cmd=1; payload must be nonempty printable ASCII
// before it is published.
inline bool RadarParseVersion(const XAtomRadarParser::Frame& f, char* out, size_t out_size) {
    if (f.func != 2 || f.cmd != 1 || f.payload_len == 0 || out == nullptr || out_size == 0) {
        return false;
    }
    for (size_t i = 0; i < f.payload_len; i++) {
        if (f.payload[i] < 0x20 || f.payload[i] > 0x7e) {
            return false;
        }
    }
    const size_t n = f.payload_len < out_size - 1 ? f.payload_len : out_size - 1;
    std::memcpy(out, f.payload, n);
    out[n] = '\0';
    return true;
}

struct RadarTelemetry {
    uint8_t state = 0;
    uint16_t distance_raw = 0;
    uint16_t signal_raw = 0;
};

// Telemetry: func=3, cmd=0x8100, payload>=10.
inline bool RadarParseTelemetry(const XAtomRadarParser::Frame& f, RadarTelemetry* t) {
    if (f.func != 3 || f.cmd != 0x8100 || f.payload_len < 10 || t == nullptr) {
        return false;
    }
    t->state = f.payload[1];
    t->distance_raw = static_cast<uint16_t>((f.payload[2] << 8) | f.payload[3]);
    t->signal_raw = static_cast<uint16_t>((f.payload[8] << 8) | f.payload[9]);
    return true;
}

// Exact version request frame from the stock finding.
inline const uint8_t* RadarVersionRequest(size_t* len) {
    static const uint8_t req[] = {0xFF, 0xFF, 0x55, 0x5A, 0x00, 0x04, 0x00, 0x00, 0x01, 0xB4};
    *len = sizeof(req);
    return req;
}

}  // namespace xatom
