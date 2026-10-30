#pragma once

#include <cassert>
#include <cstring>
#include <vector>
#include "esp_blufi_api.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psa/crypto.h"
#include "wifi_manager.h"

#include <mutex>
#include <optional>

#include "blufi_provisioning_session.h"

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_gap_ble_api.h"
#endif

class Blufi {
public:
    /**
     * @brief Get the singleton instance of the Blufi class.
     */
    static Blufi& GetInstance();

    /**
     * @brief Start WiFi scan for Blufi provisioning
     * This method intelligently handles WiFi scanning based on current WiFi state:
     * - If WiFi config mode is active, it uses the existing scan results from WifiConfigurationAp
     * - Otherwise, it performs a dedicated scan without interfering with normal WiFi operations
     * @return true if a scan was started (or was already in progress); false on failure.
     */
    bool start_wifi_scan();

    /**
     * @brief Initializes the Bluetooth controller, host, and Blufi profile.
     * This is the main entry point to start the Blufi process.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t init();

    /**
     * @brief Deinitializes Blufi and the Bluetooth stack.
     * @return ESP_OK on success, otherwise an error code.
     */
    esp_err_t deinit();

    // Delete copy constructor and assignment operator for singleton
    Blufi(const Blufi&) = delete;

    Blufi& operator=(const Blufi&) = delete;

private:
    bool inited_ = false;

    Blufi();

    ~Blufi();

    // Initialization logic
    static esp_err_t _controller_init();

    static esp_err_t _controller_deinit();

    static esp_err_t _host_init();

    static esp_err_t _host_deinit();

    static esp_err_t _gap_register_callback();

    static esp_err_t _host_and_cb_init();

    void _security_init();

    void _security_deinit();

    void _dh_negotiate_data_handler(uint8_t* data, int len, uint8_t** output_data, int* output_len,
                                    bool* need_free);

    int _aes_encrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len);

    int _aes_decrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len);

    static uint16_t _crc_checksum(uint8_t iv8, uint8_t* data, int len);

    void _handle_event(esp_blufi_cb_event_t event, esp_blufi_cb_param_t* param);

    static int _get_softap_conn_num();

    // WiFi scan methods
    void _send_wifi_list();
    void _start_dedicated_wifi_scan();
    static void _wifi_scan_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                         void* event_data);
    // Registers the persistent SCAN_DONE handler exactly once per singleton.
    esp_err_t _ensure_scan_handler_registered();
    // GET_WIFI_LIST dispatch (single response per request).
    void _handle_get_wifi_list();
    // Revoke dedicated-scan ownership before a connect attempt takes the
    // radio: clears flags, bumps the generation so an in-flight callback
    // becomes a no-op, stops our scan, and releases a deferred list request
    // with exactly one error.
    void _takeover_radio_for_connect();
    // On a claimed close: revoke scan ownership/generation and clear the
    // pending request without emitting Bluetooth frames.
    void _release_scan_for_close();

    // Provisioning connect worker bound to one session attempt.
    struct ConnectWorkerCtx;
    static void _connect_worker(void* ctx);

#ifdef CONFIG_BT_BLUEDROID_ENABLED
    // Custom discovery path used when CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
    // moves the full device name to the scan response.
    static void _gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    esp_err_t _start_mac_name_advertising();
    esp_ble_adv_params_t _adv_params = {};
    blufi_detail::BlufiAdvTracker _adv_tracker = {};
#endif

    // These C-style functions are registered with ESP-IDF and call the corresponding instance
    // methods.

    static void _event_callback_trampoline(esp_blufi_cb_event_t event, esp_blufi_cb_param_t* param);

    static void _negotiate_data_handler_trampoline(uint8_t* data, int len, uint8_t** output_data,
                                                   int* output_len, bool* need_free);

    static int _encrypt_func_trampoline(uint8_t iv8, uint8_t* crypt_data, int crypt_len);

    static int _decrypt_func_trampoline(uint8_t iv8, uint8_t* crypt_data, int crypt_len);

    static uint16_t _checksum_func_trampoline(uint8_t iv8, uint8_t* data, int len);

#ifdef CONFIG_BT_NIMBLE_ENABLED
    static void _nimble_on_reset(int reason);
    static void _nimble_on_sync();
    static void _nimble_host_task(void* param);
#endif

    // Security context, formerly blufi_sec struct
    struct BlufiSecurity {
#define DH_PARAM_LEN_MAX 1024
#define DH_SELF_PUB_KEY_LEN 384
        uint8_t self_public_key[DH_SELF_PUB_KEY_LEN];
#define SHARE_KEY_LEN 384
        uint8_t share_key[SHARE_KEY_LEN];
        size_t share_len;
#define PSK_LEN 32
        uint8_t psk[PSK_LEN];
        uint8_t* dh_param;
        int dh_param_len;
        psa_key_id_t aes_key;
        psa_cipher_operation_t enc_operation;
        psa_cipher_operation_t dec_operation;
    };

    BlufiSecurity* m_sec;

    // Provisioning session lifecycle (owns connect/report/close coordination)
    BlufiProvisioningSession m_session;
    std::optional<BlufiProvisioningSession::Attempt> m_active_attempt;

    // State variables (Bluetooth callback-owned status snapshots only; the
    // connect worker never writes these — it reports from locals).
    wifi_config_t m_sta_config{};
    int m_sta_config_ssid_len = 0;
    int m_sta_config_passwd_len = -1;
    uint8_t m_sta_bssid[6]{};
    uint8_t m_sta_ssid[32]{};
    int m_sta_ssid_len;
    esp_blufi_extra_info_t m_sta_conn_info{};

    // WiFi scan related
    std::mutex m_scan_mutex;
    std::vector<wifi_ap_record_t> m_ap_records;
    bool m_scan_in_progress = false;
    // Incremented whenever dedicated-scan ownership is revoked; a scan-done
    // callback snapshots it and revalidates before mutating shared state, so
    // a revoked callback cannot clear a newer scan or pending request.
    uint32_t m_scan_generation = 0;
    esp_event_handler_instance_t m_scan_event_instance = nullptr;
    // When true, scan results are stored in m_ap_records on scan completion.
    // Cleared during connect-to-AP so that the connect-time scan does not
    // overwrite the cache with results gathered for connection purposes.
    bool m_scan_should_save_ssid = true;
    // When true, the next scan-done event responds to a pending GET_WIFI_LIST
    // request from the App. Set by the GET_WIFI_LIST handler when no cache is
    // available or a scan is already in flight; cleared by the scan-done
    // handler after dispatching the response.
    bool m_send_list_after_scan = false;
};
