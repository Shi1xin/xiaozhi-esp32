#ifndef _XATOM_PERIPHERALS_H_
#define _XATOM_PERIPHERALS_H_

/*
 * XAtomPeripherals — board-local owners for the shared I2C1 bus, AXP2101
 * PMIC, QMI8658 IMU, RKB1161 radar UART, and raw GPIO inputs. A single 20ms
 * worker drains bounded radar chunks + polls the IMU; the PMIC samples once
 * per 1000ms. SDK calls never run inside the snapshot mutex; missing or
 * misbehaving optional devices log a real error and stay invalid without
 * aborting audio/networking.
 */

#include <atomic>
#include <functional>
#include <mutex>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_err.h>

#include "xatom_measurements.h"

struct XAtomRadarSnapshot {
    bool available = false;
    bool valid = false;  // telemetry decoded; version replies do NOT set it
    bool version_received = false;
    int last_error = 0;
    int64_t sample_us = 0;
    uint8_t state = 0;
    uint16_t distance_raw = 0;
    uint16_t signal_raw = 0;
    char version[48] = {};
    int io_level = 0;  // GPIO16 raw input
};

struct XAtomInputSnapshot {
    int button = 1;
    int radar_io = 0;
    int imu_irq = 0;
};

class XAtomPeripherals {
public:
    XAtomPeripherals() = default;
    ~XAtomPeripherals() { Stop(); }

    // Creates the I2C1 bus (100kHz, internal pulls), probes/configures all
    // optional devices, installs UART2, and starts the worker. Individual
    // device failures only mark that capability unavailable.
    esp_err_t Start();
    void Stop();

    i2c_master_bus_handle_t I2cBus() const { return i2c_bus_; }

    // Stock shutdown sequence: PMIC reg 0x10 read-modify-write bit0 set.
    // Call only from the explicit user shutdown path.
    void PmicPowerOff();

    xatom::PmicSnapshot PowerSnapshot() const;
    xatom::ImuSample MotionSnapshot() const;
    XAtomRadarSnapshot RadarSnapshot() const;
    XAtomInputSnapshot InputSnapshot() const;

    // Radar wake: a valid nonzero state may request a wake at most once per
    // 1000ms while the pending latch is clear. The latch is reset only via
    // WakeAcknowledged() from the application callback path.
    bool ConsumeRadarWakeRequest() {
        return radar_wake_pending_.exchange(false, std::memory_order_acq_rel);
    }
    void WakeAcknowledged() { radar_wake_pending_.store(false, std::memory_order_release); }
    // Fires once per newly-latched radar wake (peripheral worker context —
    // the callback must only request activity, never block).
    void SetWakeCallback(std::function<void()> cb) { wake_callback_ = std::move(cb); }

private:
    void WorkerLoop();
    void InitPmic(i2c_master_dev_handle_t* dev);
    void InitImu(i2c_master_dev_handle_t* dev);
    void SamplePmic();
    void SampleImu();
    void DrainRadar();
    void SendRadarVersionRequest();
    void HandleRadarFrame(const xatom::XAtomRadarParser::Frame& frame);
    bool RegRead(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t* out, size_t len);

    mutable std::mutex mutex_;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    i2c_master_dev_handle_t pmic_dev_ = nullptr;
    i2c_master_dev_handle_t imu_dev_ = nullptr;
    TaskHandle_t worker_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopped_{true};  // set when the worker exits its loop

    xatom::PmicSnapshot pmic_;
    xatom::ImuSample imu_;
    XAtomRadarSnapshot radar_;
    XAtomInputSnapshot inputs_;
    xatom::XAtomRadarParser parser_;

    int64_t next_pmic_us_ = 0;  // monotonic 1000ms deadline
    int version_attempts_ = 0;
    int64_t next_version_us_ = 0;
    int64_t last_radar_wake_us_ = 0;
    std::atomic<bool> radar_wake_pending_{false};
    std::function<void()> wake_callback_;
};

#endif  // _XATOM_PERIPHERALS_H_
