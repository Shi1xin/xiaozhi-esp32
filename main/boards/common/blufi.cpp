#include "blufi.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <new>
#include <string>
#include <vector>
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "wifi_manager.h"

#define BLUFI_DEVICE_NAME "Xiaozhi-Blufi"

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#endif

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "console/console.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
extern void esp_blufi_gatt_svr_register_cb(struct ble_gatt_register_ctxt* ctxt, void* arg);
extern int esp_blufi_gatt_svr_init(void);
extern void esp_blufi_gatt_svr_deinit(void);
extern void esp_blufi_btc_init(void);
extern void esp_blufi_btc_deinit(void);
#endif

extern "C" {
void esp_blufi_adv_start(void);

void esp_blufi_adv_stop(void);

void esp_blufi_disconnect(void);

void btc_blufi_report_error(esp_blufi_error_state_t state);

#ifdef CONFIG_BT_BLUEDROID_ENABLED
void esp_blufi_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
#endif

#ifdef CONFIG_BT_NIMBLE_ENABLED
void esp_blufi_gatt_svr_register_cb(struct ble_gatt_register_ctxt* ctxt, void* arg);
int esp_blufi_gatt_svr_init(void);
void esp_blufi_gatt_svr_deinit(void);
void esp_blufi_btc_init(void);
void esp_blufi_btc_deinit(void);
#endif
}

#include <wifi_station.h>
#include "esp_crc.h"
#include "ssid_manager.h"

static const char* BLUFI_TAG = "BLUFI_CLASS";

static wifi_mode_t GetWifiModeWithFallback(const WifiManager& wifi) {
    if (wifi.IsConfigMode()) {
        return WIFI_MODE_AP;
    }
    if (wifi.IsInitialized() && wifi.IsConnected()) {
        return WIFI_MODE_STA;
    }

    wifi_mode_t mode = WIFI_MODE_STA;
    esp_wifi_get_mode(&mode);
    return mode;
}

Blufi& Blufi::GetInstance() {
    static Blufi instance;
    return instance;
}

Blufi::Blufi() : m_sec(nullptr), m_sta_ssid_len(0) {
    memset(&m_sta_config, 0, sizeof(m_sta_config));
    memset(m_sta_bssid, 0, sizeof(m_sta_bssid));
    memset(m_sta_ssid, 0, sizeof(m_sta_ssid));
    memset(&m_sta_conn_info, 0, sizeof(m_sta_conn_info));
}

Blufi::~Blufi() {
    if (m_sec) {
        _security_deinit();
    }
}

esp_err_t Blufi::init() {
    esp_err_t ret = ESP_FAIL;
    inited_ = true;
    m_session.Reset();
    m_active_attempt.reset();

    // The persistent scan handler needs the default event loop, which
    // WifiManager::Initialize() installs — ensure it before registering.
    auto& wifi_manager = WifiManager::GetInstance();
    if (!wifi_manager.IsInitialized() && !wifi_manager.Initialize()) {
        ESP_LOGE(BLUFI_TAG, "Failed to initialize WifiManager");
        return ESP_FAIL;
    }

    // One persistent scan-done registration per singleton, before any mode
    // branch below. It stays registered through deinit so a pending scan can
    // always complete; handlers must not send Bluetooth frames after close.
    ret = _ensure_scan_handler_registered();
    if (ret != ESP_OK) {
        ESP_LOGE(BLUFI_TAG, "Failed to register scan handler: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start WiFi scan early to have results ready when user connects
    if (!wifi_manager.IsInitialized() || !wifi_manager.IsConfigMode()) {
        // start scan immediately
        start_wifi_scan();
    } else {
        ESP_LOGE(BLUFI_TAG,
                 "Blufi and WiFi hotspot network configuration cannot "
                 "be used simultaneously.");
        return ret;
    }

#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
    ret = _controller_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "BLUFI controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
#endif

    ret = _host_and_cb_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "BLUFI host and cb init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(BLUFI_TAG, "BLUFI VERSION %04x", esp_blufi_get_version());
    return ESP_OK;
}

esp_err_t Blufi::deinit() {
    esp_err_t ret = ESP_OK;

    if (inited_) {
        // Claim the close before touching the SDK. While a connect attempt or
        // its success-report window is still owned by a worker the teardown is
        // deferred to that worker's FinishReport -> deinit() call.
        switch (m_session.TryBeginDeinit()) {
            case BlufiProvisioningSession::DeinitResult::AlreadyStarted:
                return ESP_OK;
            case BlufiProvisioningSession::DeinitResult::Deferred:
                ESP_LOGI(BLUFI_TAG, "deinit deferred until active provisioning finishes");
                return ESP_OK;
            case BlufiProvisioningSession::DeinitResult::Proceed:
                break;
        }
        // Claimed close: revoke scan ownership and drop a deferred list
        // request silently — no Bluetooth frames go out from here.
        _release_scan_for_close();
        if (m_session.IsBleConnected()) {
            esp_blufi_disconnect();
        }
        ret = _host_deinit();
        if (ret) {
            ESP_LOGE(BLUFI_TAG, "Host deinit failed: %s", esp_err_to_name(ret));
        }
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
        ret = _controller_deinit();
        if (ret) {
            ESP_LOGE(BLUFI_TAG, "Controller deinit failed: %s", esp_err_to_name(ret));
        }
#endif
    }
    return ret;
}

#ifdef CONFIG_BT_BLUEDROID_ENABLED
esp_err_t Blufi::_host_init() {
    esp_err_t ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ESP_LOGI(BLUFI_TAG, "BD ADDR: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
    return ESP_OK;
}

esp_err_t Blufi::_host_deinit() {
    esp_err_t ret = esp_blufi_profile_deinit();
    if (ret != ESP_OK)
        return ret;

    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s disable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    ret = esp_bluedroid_deinit();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s deinit bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t Blufi::_gap_register_callback() {
#ifdef CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
    esp_err_t rc = esp_ble_gap_register_callback(&Blufi::_gap_event_handler);
#else
    esp_err_t rc = esp_ble_gap_register_callback(esp_blufi_gap_event_handler);
#endif
    if (rc) {
        return rc;
    }
    return esp_blufi_profile_init();
}

esp_err_t Blufi::_host_and_cb_init() {
    static esp_blufi_callbacks_t blufi_callbacks = {
        .event_cb = &_event_callback_trampoline,
        .negotiate_data_handler = &_negotiate_data_handler_trampoline,
        .encrypt_func = &_encrypt_func_trampoline,
        .decrypt_func = &_decrypt_func_trampoline,
        .checksum_func = &_checksum_func_trampoline,
    };

    esp_err_t ret = _host_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s initialise host failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s blufi register failed, error code = %x", __func__, ret);
        return ret;
    }
    ret = _gap_register_callback();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return ret;
    }
    return ESP_OK;
}
#endif /* CONFIG_BT_BLUEDROID_ENABLED */

#ifdef CONFIG_BT_NIMBLE_ENABLED
// Stubs for NimBLE specific store functionality
void ble_store_config_init();

void Blufi::_nimble_on_reset(int reason) {
    ESP_LOGE(BLUFI_TAG, "NimBLE Resetting state; reason=%d", reason);
}

void Blufi::_nimble_on_sync() { esp_blufi_profile_init(); }

void Blufi::_nimble_host_task(void* param) {
    ESP_LOGI(BLUFI_TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t Blufi::_host_init() {
    ble_hs_cfg.reset_cb = _nimble_on_reset;
    ble_hs_cfg.sync_cb = _nimble_on_sync;
    ble_hs_cfg.gatts_register_cb = esp_blufi_gatt_svr_register_cb;

    ble_hs_cfg.sm_io_cap = 4;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
#endif

    int rc = esp_blufi_gatt_svr_init();
    assert(rc == 0);

    ble_store_config_init();
    esp_blufi_btc_init();

    esp_err_t err = esp_nimble_enable(_nimble_host_task);
    if (err) {
        ESP_LOGE(BLUFI_TAG, "%s failed: %s", __func__, esp_err_to_name(err));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t Blufi::_host_deinit(void) {
    esp_err_t ret = nimble_port_stop();
    if (ret == ESP_OK) {
        esp_nimble_deinit();
    }
    esp_blufi_gatt_svr_deinit();
    ret = esp_blufi_profile_deinit();
    esp_blufi_btc_deinit();
    return ret;
}

esp_err_t Blufi::_gap_register_callback(void) { return ESP_OK; }

esp_err_t Blufi::_host_and_cb_init() {
    static esp_blufi_callbacks_t blufi_callbacks = {
        .event_cb = &_event_callback_trampoline,
        .negotiate_data_handler = &_negotiate_data_handler_trampoline,
        .encrypt_func = &_encrypt_func_trampoline,
        .decrypt_func = &_decrypt_func_trampoline,
        .checksum_func = &_checksum_func_trampoline,
    };

    esp_err_t ret = esp_blufi_register_callbacks(&blufi_callbacks);
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s blufi register failed, error code = %x", __func__, ret);
        return ret;
    }

    // Host init must be called after registering callbacks for NimBLE
    ret = _host_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s initialise host failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
#endif /* CONFIG_BT_NIMBLE_ENABLED */

#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
esp_err_t Blufi::_controller_init() {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }

#ifdef CONFIG_BT_NIMBLE_ENABLED
    ret = esp_nimble_init();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "esp_nimble_init() failed: %s", esp_err_to_name(ret));
        return ret;
    }
#endif
    return ESP_OK;
}

esp_err_t Blufi::_controller_deinit() {
    esp_err_t ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s disable controller failed: %s", __func__, esp_err_to_name(ret));
    }
    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(BLUFI_TAG, "%s deinit controller failed: %s", __func__, esp_err_to_name(ret));
    }
    return ret;
}
#endif

namespace {

constexpr uint8_t kDhParamLength = 0x00;
constexpr uint8_t kDhParamData = 0x01;
constexpr size_t kBlufiIvSize = 16;
constexpr size_t kBlufiHashSize = 32;
constexpr char kBlufiEncryptDomain[] = "blufi_enc";
constexpr char kBlufiDecryptDomain[] = "blufi_dec";
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
constexpr auto kBlufiHashError = ESP_BLUFI_CALC_SHA_256_ERROR;
#else
// IDF 5 does not expose a SHA-256-specific BluFi error code.
constexpr auto kBlufiHashError = ESP_BLUFI_CALC_MD5_ERROR;
#endif

}  // namespace

void Blufi::_security_init() {
    _security_deinit();

    psa_status_t status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(BLUFI_TAG, "psa_crypto_init failed: %d", status);
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    m_sec = new BlufiSecurity();
    if (m_sec == nullptr) {
        ESP_LOGE(BLUFI_TAG, "Failed to allocate security context");
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }
    memset(m_sec, 0, sizeof(BlufiSecurity));
    m_sec->enc_operation = psa_cipher_operation_init();
    m_sec->dec_operation = psa_cipher_operation_init();
}

void Blufi::_security_deinit() {
    if (m_sec == nullptr) {
        return;
    }

    psa_cipher_abort(&m_sec->enc_operation);
    psa_cipher_abort(&m_sec->dec_operation);
    if (m_sec->aes_key != PSA_KEY_ID_NULL) {
        psa_destroy_key(m_sec->aes_key);
    }
    free(m_sec->dh_param);
    memset(m_sec, 0, sizeof(BlufiSecurity));
    delete m_sec;
    m_sec = nullptr;
}

void Blufi::_dh_negotiate_data_handler(uint8_t* data, int len, uint8_t** output_data,
                                       int* output_len, bool* need_free) {
    if (m_sec == nullptr || data == nullptr || output_data == nullptr || output_len == nullptr ||
        need_free == nullptr) {
        ESP_LOGE(BLUFI_TAG, "Security not initialized in DH handler");
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    if (len < 3) {
        ESP_LOGE(BLUFI_TAG, "DH handler: data too short");
        btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
        return;
    }

    uint8_t type = data[0];
    switch (type) {
        case kDhParamLength:
            m_sec->dh_param_len = (data[1] << 8) | data[2];
            if (m_sec->dh_param_len <= 0 || m_sec->dh_param_len > DH_PARAM_LEN_MAX) {
                ESP_LOGE(BLUFI_TAG, "Invalid DH parameter length: %d", m_sec->dh_param_len);
                m_sec->dh_param_len = 0;
                btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                return;
            }

            free(m_sec->dh_param);
            m_sec->dh_param = nullptr;
            psa_cipher_abort(&m_sec->enc_operation);
            psa_cipher_abort(&m_sec->dec_operation);
            m_sec->enc_operation = psa_cipher_operation_init();
            m_sec->dec_operation = psa_cipher_operation_init();
            if (m_sec->aes_key != PSA_KEY_ID_NULL) {
                psa_destroy_key(m_sec->aes_key);
                m_sec->aes_key = PSA_KEY_ID_NULL;
            }

            m_sec->dh_param = (uint8_t*)malloc(m_sec->dh_param_len);
            if (m_sec->dh_param == nullptr) {
                ESP_LOGE(BLUFI_TAG, "DH malloc failed");
                m_sec->dh_param_len = 0;
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                return;
            }
            break;
        case kDhParamData: {
            if (m_sec->dh_param == nullptr) {
                ESP_LOGE(BLUFI_TAG, "DH param not allocated");
                btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                return;
            }
            if (len != m_sec->dh_param_len + 1) {
                ESP_LOGE(BLUFI_TAG, "Invalid DH parameter packet length: %d, expected %d", len,
                         m_sec->dh_param_len + 1);
                btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                return;
            }

            memcpy(m_sec->dh_param, &data[1], m_sec->dh_param_len);

            const uint8_t* param = m_sec->dh_param;
            const uint8_t* const end = param + m_sec->dh_param_len;
            auto read_field = [&param, end](const uint8_t** value, size_t* value_len) -> bool {
                if (end - param < 2) {
                    return false;
                }
                *value_len = (static_cast<size_t>(param[0]) << 8) | param[1];
                param += 2;
                if (*value_len == 0 || static_cast<size_t>(end - param) < *value_len) {
                    return false;
                }
                *value = param;
                param += *value_len;
                return true;
            };

            const uint8_t* prime = nullptr;
            const uint8_t* generator = nullptr;
            const uint8_t* peer_public_key = nullptr;
            size_t prime_len = 0;
            size_t generator_len = 0;
            size_t peer_public_key_len = 0;
            if (!read_field(&prime, &prime_len) || !read_field(&generator, &generator_len) ||
                !read_field(&peer_public_key, &peer_public_key_len) || param != end) {
                ESP_LOGE(BLUFI_TAG, "Malformed DH parameter data");
                btc_blufi_report_error(ESP_BLUFI_READ_PARAM_ERROR);
                return;
            }

            // PSA FFDH supports named RFC 7919 groups instead of caller-supplied P/G. The IDF 6
            // BluFi protocol uses ffdhe3072; P and G remain in the packet for framing
            // compatibility.
            constexpr size_t kDhKeyBits = 3072;
            constexpr size_t kDhKeyBytes = kDhKeyBits / 8;
            if (prime_len != kDhKeyBytes || peer_public_key_len != kDhKeyBytes ||
                generator_len != 1 || generator[0] != 2) {
                ESP_LOGE(BLUFI_TAG,
                         "Unsupported DH group (P=%u, G=%u, public=%u); ffdhe3072 is required",
                         static_cast<unsigned>(prime_len), static_cast<unsigned>(generator_len),
                         static_cast<unsigned>(peer_public_key_len));
                btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
                return;
            }

            psa_key_attributes_t attributes = psa_key_attributes_init();
            psa_set_key_type(&attributes, PSA_KEY_TYPE_DH_KEY_PAIR(PSA_DH_FAMILY_RFC7919));
            psa_set_key_bits(&attributes, kDhKeyBits);
            psa_set_key_algorithm(&attributes, PSA_ALG_FFDH);
            psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE);

            psa_key_id_t private_key = PSA_KEY_ID_NULL;
            psa_status_t status = psa_generate_key(&attributes, &private_key);
            psa_reset_key_attributes(&attributes);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(BLUFI_TAG, "psa_generate_key failed: %d", status);
                btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
                return;
            }

            size_t public_key_len = 0;
            status = psa_export_public_key(private_key, m_sec->self_public_key,
                                           sizeof(m_sec->self_public_key), &public_key_len);
            if (status != PSA_SUCCESS || public_key_len != kDhKeyBytes) {
                ESP_LOGE(BLUFI_TAG, "psa_export_public_key failed: %d, length: %u", status,
                         static_cast<unsigned>(public_key_len));
                psa_destroy_key(private_key);
                btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
                return;
            }

            status = psa_raw_key_agreement(PSA_ALG_FFDH, private_key, peer_public_key,
                                           peer_public_key_len, m_sec->share_key,
                                           sizeof(m_sec->share_key), &m_sec->share_len);
            psa_destroy_key(private_key);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(BLUFI_TAG, "psa_raw_key_agreement failed: %d", status);
                btc_blufi_report_error(ESP_BLUFI_ENCRYPT_ERROR);
                return;
            }

            size_t hash_len = 0;
            status = psa_hash_compute(PSA_ALG_SHA_256, m_sec->share_key, m_sec->share_len,
                                      m_sec->psk, sizeof(m_sec->psk), &hash_len);
            if (status != PSA_SUCCESS || hash_len != sizeof(m_sec->psk)) {
                ESP_LOGE(BLUFI_TAG, "psa_hash_compute failed: %d", status);
                btc_blufi_report_error(kBlufiHashError);
                return;
            }

            attributes = psa_key_attributes_init();
            psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
            psa_set_key_bits(&attributes, PSK_LEN * 8);
            psa_set_key_algorithm(&attributes, PSA_ALG_CTR);
            psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
            status = psa_import_key(&attributes, m_sec->psk, sizeof(m_sec->psk), &m_sec->aes_key);
            psa_reset_key_attributes(&attributes);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(BLUFI_TAG, "psa_import_key failed: %d", status);
                btc_blufi_report_error(ESP_BLUFI_ENCRYPT_ERROR);
                return;
            }

            auto setup_cipher = [this](psa_cipher_operation_t* operation, const char* domain) {
                uint8_t material[sizeof(kBlufiEncryptDomain) - 1 + SHARE_KEY_LEN];
                memcpy(material, domain, sizeof(kBlufiEncryptDomain) - 1);
                memcpy(material + sizeof(kBlufiEncryptDomain) - 1, m_sec->share_key,
                       m_sec->share_len);
                uint8_t hash[kBlufiHashSize];
                size_t hash_len = 0;
                psa_status_t status = psa_hash_compute(
                    PSA_ALG_SHA_256, material, sizeof(kBlufiEncryptDomain) - 1 + m_sec->share_len,
                    hash, sizeof(hash), &hash_len);
                memset(material, 0, sizeof(material));
                if (status != PSA_SUCCESS || hash_len != sizeof(hash)) {
                    return status == PSA_SUCCESS ? PSA_ERROR_GENERIC_ERROR : status;
                }
                *operation = psa_cipher_operation_init();
                status = psa_cipher_encrypt_setup(operation, m_sec->aes_key, PSA_ALG_CTR);
                if (status == PSA_SUCCESS) {
                    status = psa_cipher_set_iv(operation, hash, kBlufiIvSize);
                }
                memset(hash, 0, sizeof(hash));
                return status;
            };

            status = setup_cipher(&m_sec->enc_operation, kBlufiEncryptDomain);
            if (status == PSA_SUCCESS) {
                // CTR decryption uses the same cipher primitive with an independent counter.
                status = setup_cipher(&m_sec->dec_operation, kBlufiDecryptDomain);
            }
            if (status != PSA_SUCCESS) {
                ESP_LOGE(BLUFI_TAG, "PSA cipher setup failed: %d", status);
                psa_cipher_abort(&m_sec->enc_operation);
                psa_cipher_abort(&m_sec->dec_operation);
                psa_destroy_key(m_sec->aes_key);
                m_sec->aes_key = PSA_KEY_ID_NULL;
                btc_blufi_report_error(ESP_BLUFI_ENCRYPT_ERROR);
                return;
            }

            *output_data = m_sec->self_public_key;
            *output_len = public_key_len;
            *need_free = false;
            ESP_LOGI(BLUFI_TAG, "DH negotiation completed successfully");

            free(m_sec->dh_param);
            m_sec->dh_param = nullptr;
            m_sec->dh_param_len = 0;
            break;
        }
        default:
            ESP_LOGE(BLUFI_TAG, "DH handler unknown type: %d", type);
            btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
    }
}

int Blufi::_aes_encrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len) {
    (void)iv8;
    if (!m_sec || m_sec->aes_key == PSA_KEY_ID_NULL || !crypt_data || crypt_len < 0) {
        ESP_LOGE(BLUFI_TAG, "Invalid parameters for AES encryption");
        return -ESP_ERR_INVALID_ARG;
    }
    if (crypt_len == 0) {
        return 0;
    }

    std::vector<uint8_t> output(PSA_CIPHER_ENCRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES, PSA_ALG_CTR,
                                                               static_cast<size_t>(crypt_len)));
    size_t output_len = 0;
    psa_status_t status = psa_cipher_update(&m_sec->enc_operation, crypt_data, crypt_len,
                                            output.data(), output.size(), &output_len);
    if (status != PSA_SUCCESS || output_len != static_cast<size_t>(crypt_len)) {
        ESP_LOGE(BLUFI_TAG, "AES encrypt failed: %d, length: %u", status,
                 static_cast<unsigned>(output_len));
        return status == PSA_SUCCESS ? -ESP_FAIL : status;
    }
    memcpy(crypt_data, output.data(), output_len);
    return static_cast<int>(output_len);
}

int Blufi::_aes_decrypt(uint8_t iv8, uint8_t* crypt_data, int crypt_len) {
    (void)iv8;
    if (!m_sec || m_sec->aes_key == PSA_KEY_ID_NULL || !crypt_data || crypt_len < 0) {
        ESP_LOGE(BLUFI_TAG, "Invalid parameters for AES decryption");
        return -ESP_ERR_INVALID_ARG;
    }
    if (crypt_len == 0) {
        return 0;
    }

    std::vector<uint8_t> output(PSA_CIPHER_DECRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES, PSA_ALG_CTR,
                                                               static_cast<size_t>(crypt_len)));
    size_t output_len = 0;
    psa_status_t status = psa_cipher_update(&m_sec->dec_operation, crypt_data, crypt_len,
                                            output.data(), output.size(), &output_len);
    if (status != PSA_SUCCESS || output_len != static_cast<size_t>(crypt_len)) {
        ESP_LOGE(BLUFI_TAG, "AES decrypt failed: %d, length: %u", status,
                 static_cast<unsigned>(output_len));
        return status == PSA_SUCCESS ? -ESP_FAIL : status;
    }
    memcpy(crypt_data, output.data(), output_len);
    return static_cast<int>(output_len);
}

uint16_t Blufi::_crc_checksum(uint8_t iv8, uint8_t* data, int len) {
    return esp_crc16_be(0, data, len);
}

int Blufi::_get_softap_conn_num() {
    auto& wifi = WifiManager::GetInstance();
    if (!wifi.IsInitialized() || !wifi.IsConfigMode()) {
        return 0;
    }

    wifi_sta_list_t sta_list{};
    if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
        return sta_list.num;
    }
    return 0;
}

bool Blufi::start_wifi_scan() {
    ESP_LOGI(BLUFI_TAG, "Starting dedicated WiFi scan");

    // While a connect attempt or a claimed close owns the radio this scan
    // must not touch Wi-Fi APIs at all. A deferred list request is released
    // with at most one error under the usual connected/not-closing rule.
    if (m_session.IsConnecting() || m_session.IsClosing()) {
        bool pending = false;
        {
            std::lock_guard<std::mutex> lock(m_scan_mutex);
            pending = m_send_list_after_scan;
            m_send_list_after_scan = false;
        }
        if (blufi_detail::BlufiShouldReportScanError(pending, m_session.IsBleConnected(),
                                                     m_session.IsClosing())) {
            esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
        }
        return false;
    }

    uint32_t claimed_generation = 0;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        // Already running: caller can rely on the in-flight scan and await its done event.
        if (m_scan_in_progress) {
            ESP_LOGW(BLUFI_TAG, "Scan already in progress, skipping");
            return true;
        }
        m_scan_in_progress = true;
        // This scan owns the done-callback window; re-arm record saving since
        // a connect takeover may have cleared it.
        m_scan_should_save_ssid = true;
        claimed_generation = ++m_scan_generation;
    }

    auto fail = [this, claimed_generation]() {
        bool pending = false;
        {
            std::lock_guard<std::mutex> lock(m_scan_mutex);
            // A failed older SDK start must not clear a newer scan's
            // ownership or pending request.
            if (m_scan_in_progress && m_scan_generation == claimed_generation) {
                m_scan_in_progress = false;
                // Atomically release a request deferred on this scan, then
                // emit exactly one error if the link is still usable.
                pending = m_send_list_after_scan;
                m_send_list_after_scan = false;
            }
        }
        if (blufi_detail::BlufiShouldReportScanError(pending, m_session.IsBleConnected(),
                                                     m_session.IsClosing())) {
            esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
        }
        return false;
    };

    // True while this call still owns the dedicated-scan slot: revocation
    // (connect takeover, close, or a newer scan) must stop further SDK calls
    // without touching the newer owner's flags.
    auto owns_scan = [this, claimed_generation]() {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        return m_scan_in_progress && m_scan_generation == claimed_generation &&
               !m_session.IsConnecting() && !m_session.IsClosing();
    };

    // Check the API result before reading the mode.
    wifi_mode_t current_mode = WIFI_MODE_NULL;
    esp_err_t err = esp_wifi_get_mode(&current_mode);
    if (err != ESP_OK) {
        ESP_LOGE(BLUFI_TAG, "Failed to get WiFi mode: %s", esp_err_to_name(err));
        return fail();
    }
    if (!owns_scan()) {
        return false;
    }

    // NULL bootstraps STA, AP preserves the hotspot via APSTA, STA/APSTA scan
    // in place. The persistent SCAN_DONE handler was registered at init.
    const int target_mode = blufi_detail::BlufiScanTargetMode(static_cast<int>(current_mode));
    if (target_mode < 0) {
        ESP_LOGE(BLUFI_TAG, "Unexpected WiFi mode: %d", current_mode);
        return fail();
    }
    if (target_mode != static_cast<int>(current_mode)) {
        ESP_LOGI(BLUFI_TAG, "Switching WiFi mode %d -> %d for scan", current_mode, target_mode);
        err = esp_wifi_set_mode(static_cast<wifi_mode_t>(target_mode));
        if (err != ESP_OK) {
            ESP_LOGE(BLUFI_TAG, "Failed to set WiFi mode: %s", esp_err_to_name(err));
            return fail();
        }
    }
    if (!owns_scan()) {
        return false;
    }
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(BLUFI_TAG, "Failed to start WiFi before scan: %s", esp_err_to_name(err));
        return fail();
    }
    if (!owns_scan()) {
        return false;
    }
    err = esp_wifi_scan_start(NULL, false);
    if (err != ESP_OK) {
        ESP_LOGE(BLUFI_TAG, "Failed to start WiFi scan: %s", esp_err_to_name(err));
        return fail();
    }

    ESP_LOGI(BLUFI_TAG, "WiFi scan started");
    return true;
}

void Blufi::_send_wifi_list() {
    // Snapshot under the scan mutex; Bluetooth frames go out after release.
    std::vector<wifi_ap_record_t> snapshot;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        snapshot = m_ap_records;
        m_ap_records.clear();
    }

    if (m_session.IsClosing() || !m_session.IsBleConnected()) {
        ESP_LOGW(BLUFI_TAG, "Skipping WiFi list send: closing or disconnected");
        return;
    }
    if (snapshot.empty()) {
        ESP_LOGW(BLUFI_TAG, "No AP records available, sending WiFi scan fail");
        esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
        return;
    }

    ESP_LOGI(BLUFI_TAG, "Sending WiFi list with %d APs", snapshot.size());

    std::vector<esp_blufi_ap_record_t> blufi_ap_list;
    for (const auto& ap : snapshot) {
        esp_blufi_ap_record_t blufi_ap;
        memset(&blufi_ap, 0, sizeof(blufi_ap));
        memcpy(blufi_ap.ssid, ap.ssid, std::min((size_t)32, sizeof(ap.ssid)));
        blufi_ap.rssi = ap.rssi;
        blufi_ap_list.push_back(blufi_ap);
    }

    esp_blufi_send_wifi_list(blufi_ap_list.size(), blufi_ap_list.data());

    // Refresh the cache asynchronously for the next request — but never
    // while a connect attempt or claimed close owns the radio.
    if (!m_session.IsConnecting() && !m_session.IsClosing()) {
        start_wifi_scan();
    }
}

void Blufi::_wifi_scan_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                     void* event_data) {
    Blufi* self = static_cast<Blufi*>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        // This handler stays registered through deinit and also observes
        // WifiStation's own connect scans — only a scan this singleton owns
        // (in_progress) may touch the cache or consume records. Ownership is
        // tied to a generation so a callback whose scan was revoked mid-flight
        // (connect takeover, close) cannot clear or mutate newer state.
        uint32_t generation = 0;
        {
            std::lock_guard<std::mutex> lock(self->m_scan_mutex);
            if (!self->m_scan_in_progress) {
                return;  // station-owned scan done event
            }
            generation = self->m_scan_generation;
        }
        ESP_LOGI(BLUFI_TAG, "WiFi scan done");

        const wifi_event_sta_scan_done_t* done =
            static_cast<const wifi_event_sta_scan_done_t*>(event_data);
        // status==0 is a completed scan; nonzero (or a null event payload)
        // is failure: no count/record fetches happen in that case.
        const bool scan_ok = done != nullptr && blufi_detail::BlufiScanDoneOk(done->status);

        auto finish_failed = [self, generation]() {
            bool pending = false;
            {
                std::lock_guard<std::mutex> lock(self->m_scan_mutex);
                if (self->m_scan_in_progress && self->m_scan_generation == generation) {
                    self->m_scan_in_progress = false;
                    pending = self->m_send_list_after_scan;
                    self->m_send_list_after_scan = false;
                }
            }
            if (blufi_detail::BlufiShouldReportScanError(pending, self->m_session.IsBleConnected(),
                                                         self->m_session.IsClosing())) {
                esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
            }
        };

        if (!scan_ok) {
            ESP_LOGE(BLUFI_TAG, "WiFi scan failed or missing event data");
            finish_failed();
            return;
        }

        // SDK calls happen outside the mutex; ownership is revalidated after
        // each so a mid-flight revocation turns this callback into a no-op.
        uint16_t ap_num = 0;
        esp_err_t err = esp_wifi_scan_get_ap_num(&ap_num);
        if (err != ESP_OK) {
            ESP_LOGE(BLUFI_TAG, "Failed to get AP count: %s", esp_err_to_name(err));
            finish_failed();
            return;
        }

        bool save = false;
        {
            std::lock_guard<std::mutex> lock(self->m_scan_mutex);
            save = self->m_scan_in_progress && self->m_scan_generation == generation &&
                   self->m_scan_should_save_ssid && !self->m_session.IsClosing();
        }

        std::vector<wifi_ap_record_t> fetched;
        if (save && ap_num > 0) {
            // Bound the temporary records vector even if the driver reports
            // more APs than the cache holds.
            const uint16_t capped = static_cast<uint16_t>(blufi_detail::BlufiClampApCount(ap_num));
            fetched.resize(capped);
            uint16_t got = capped;
            err = esp_wifi_scan_get_ap_records(&got, fetched.data());
            if (err != ESP_OK) {
                ESP_LOGE(BLUFI_TAG, "Failed to get AP records: %s", esp_err_to_name(err));
                finish_failed();
                return;
            }
            fetched.resize(got);
        }

        bool dispatch = false;
        {
            std::lock_guard<std::mutex> lock(self->m_scan_mutex);
            if (self->m_scan_in_progress && self->m_scan_generation == generation) {
                // Publication revalidation: only the still-owned callback may
                // install the cache and release the deferred request.
                self->m_ap_records = std::move(fetched);
                self->m_scan_in_progress = false;
                if (self->m_send_list_after_scan) {
                    self->m_send_list_after_scan = false;
                    dispatch = true;
                }
                ESP_LOGI(BLUFI_TAG, "Found %d APs", self->m_ap_records.size());
            }
        }
        // Dispatch after releasing the lock; _send_wifi_list re-checks the
        // session so a disconnected/closing session emits no frames.
        if (dispatch) {
            self->_send_wifi_list();
        }
    }
}

esp_err_t Blufi::_ensure_scan_handler_registered() {
    if (m_scan_event_instance != nullptr) {
        return ESP_OK;  // persistent registration: exactly once per singleton
    }
    return esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
                                               &Blufi::_wifi_scan_event_handler, this,
                                               &m_scan_event_instance);
}

void Blufi::_takeover_radio_for_connect() {
    bool was_active = false;
    bool pending = false;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        was_active = m_scan_in_progress;
        m_scan_in_progress = false;
        m_scan_should_save_ssid = false;
        pending = m_send_list_after_scan;
        m_send_list_after_scan = false;
        ++m_scan_generation;  // revoke any in-flight dedicated-scan callback
    }
    // Stop only a scan this singleton actually started; never a station one.
    if (was_active) {
        esp_wifi_scan_stop();
    }
    if (blufi_detail::BlufiShouldReportScanError(pending, m_session.IsBleConnected(),
                                                 m_session.IsClosing())) {
        esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
    }
}

void Blufi::_release_scan_for_close() {
    std::lock_guard<std::mutex> lock(m_scan_mutex);
    m_scan_in_progress = false;
    m_scan_should_save_ssid = false;
    m_send_list_after_scan = false;  // pending request released silently
    ++m_scan_generation;
}

void Blufi::_handle_get_wifi_list() {
    // Defer to an in-flight scan, answer from the cache, or start a fresh
    // scan; exactly one response goes out in every path. While a connect
    // attempt owns the radio no dedicated scan is started — the cache
    // answers if populated, otherwise one error.
    blufi_detail::BlufiWifiListAction action;
    bool connecting = false;
    {
        std::lock_guard<std::mutex> lock(m_scan_mutex);
        // Capture the connect-phase decision once inside the lock; the
        // session may transition (BeginReport) before the send below.
        connecting = m_session.IsConnecting();
        if (connecting) {
            action = !m_ap_records.empty() ? blufi_detail::BlufiWifiListAction::RespondCached
                                           : blufi_detail::BlufiWifiListAction::DeferToInFlight;
        } else {
            action =
                blufi_detail::BlufiResolveWifiListAction(m_scan_in_progress, !m_ap_records.empty());
            if (action == blufi_detail::BlufiWifiListAction::DeferToInFlight ||
                action == blufi_detail::BlufiWifiListAction::StartFreshScan) {
                m_scan_should_save_ssid = true;
                m_send_list_after_scan = true;
            }
        }
    }
    switch (action) {
        case blufi_detail::BlufiWifiListAction::RespondCached:
            _send_wifi_list();
            break;
        case blufi_detail::BlufiWifiListAction::StartFreshScan:
            // A failed start already released the deferred request and
            // emitted the single error frame inside the scan failure path.
            start_wifi_scan();
            break;
        case blufi_detail::BlufiWifiListAction::DeferToInFlight:
            // The captured decision is authoritative: a connecting request
            // with no cache gets exactly one error; a genuine in-flight defer
            // (not connecting) waits for the done handler.
            if (connecting && blufi_detail::BlufiShouldReportScanError(
                                  true, m_session.IsBleConnected(), m_session.IsClosing())) {
                esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
            }
            break;
    }
}

struct Blufi::ConnectWorkerCtx {
    Blufi* self;
    BlufiProvisioningSession::Attempt token;
    std::string ssid;
};

void Blufi::_handle_event(esp_blufi_cb_event_t event, esp_blufi_cb_param_t* param) {
    switch (event) {
        case ESP_BLUFI_EVENT_INIT_FINISH:
            ESP_LOGI(BLUFI_TAG, "BLUFI init finish");
#ifdef CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
            if (_start_mac_name_advertising() != ESP_OK) {
                ESP_LOGE(BLUFI_TAG, "Failed to start MAC-name advertising");
            }
#else
            esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
            esp_blufi_adv_start();
#endif
            break;
        case ESP_BLUFI_EVENT_DEINIT_FINISH:
            ESP_LOGI(BLUFI_TAG, "BLUFI deinit finish");
            break;
        case ESP_BLUFI_EVENT_BLE_CONNECT:
            ESP_LOGI(BLUFI_TAG, "BLUFI ble connect");
            m_session.SetBleConnected(true);
            // A new BLE session must not inherit a previous session's
            // credentials — the peer resends them after connecting.
            memset(&m_sta_config, 0, sizeof(m_sta_config));
            m_sta_config_ssid_len = 0;
            m_sta_config_passwd_len = -1;
            esp_blufi_adv_stop();
            _security_init();
            break;
        case ESP_BLUFI_EVENT_BLE_DISCONNECT:
            ESP_LOGI(BLUFI_TAG, "BLUFI ble disconnect");
            m_session.SetBleConnected(false);
            _security_deinit();
            if (m_session.IsClosing()) {
                // Close was already claimed; do not advertise or schedule
                // redundant cleanup here.
                break;
            }
            if (!m_session.IsProvisioned()) {
#ifdef CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
                _start_mac_name_advertising();
#else
                esp_blufi_adv_start();
#endif
            } else {
                esp_blufi_adv_stop();
                xTaskCreate(
                    [](void* ctx) {
                        static_cast<Blufi*>(ctx)->deinit();
                        vTaskDelete(nullptr);
                    },
                    "blufi_deinit", 4096, this, 5, nullptr);
            }
            break;
        case ESP_BLUFI_EVENT_SET_WIFI_OPMODE: {
            ESP_LOGI(BLUFI_TAG, "BLUFI Set WIFI opmode %d", param->wifi_mode.op_mode);
            auto& wifi_manager = WifiManager::GetInstance();
            if (!wifi_manager.IsInitialized() && !wifi_manager.Initialize()) {
                ESP_LOGE(BLUFI_TAG, "Failed to initialize WifiManager for opmode change");
                break;
            }
            switch (param->wifi_mode.op_mode) {
                case WIFI_MODE_STA: {
                    // Set the STA role only; the explicit connect request owns
                    // StartStation so an opmode frame never auto-connects to
                    // previously stored credentials.
                    const esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
                    if (err != ESP_OK) {
                        ESP_LOGW(BLUFI_TAG, "set WIFI_MODE_STA failed: %s", esp_err_to_name(err));
                    }
                    break;
                }
                case WIFI_MODE_AP:
                    wifi_manager.StartConfigAp();
                    break;
                case WIFI_MODE_APSTA: {
                    ESP_LOGW(BLUFI_TAG, "APSTA mode: setting role without starting station");
                    const esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
                    if (err != ESP_OK) {
                        ESP_LOGW(BLUFI_TAG, "set WIFI_MODE_APSTA failed: %s", esp_err_to_name(err));
                    }
                    break;
                }
                default:
                    wifi_manager.StopStation();
                    wifi_manager.StopConfigAp();
                    break;
            }
            break;
        }
        case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP: {
            ESP_LOGI(BLUFI_TAG, "BLUFI request wifi connect to AP via esp-wifi-connect");
            // Validate the stored credential lengths before constructing any
            // string or touching NVS/station state — a missing or malformed
            // credential frame makes this request unanswerable.
            if (!blufi_detail::BlufiValidSsidLen(static_cast<size_t>(m_sta_config_ssid_len)) ||
                !blufi_detail::BlufiValidPasswordLen(
                    static_cast<size_t>(m_sta_config_passwd_len))) {
                ESP_LOGE(BLUFI_TAG,
                         "Connect rejected: invalid credentials (ssid_len=%d passwd_len=%d)",
                         m_sta_config_ssid_len, m_sta_config_passwd_len);
                btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
                break;
            }
            std::string ssid(reinterpret_cast<const char*>(m_sta_config.sta.ssid),
                             static_cast<size_t>(m_sta_config_ssid_len));
            std::string password(reinterpret_cast<const char*>(m_sta_config.sta.password),
                                 static_cast<size_t>(m_sta_config_passwd_len));

            // New session attempt BEFORE touching the station; a retry
            // atomically invalidates the previous attempt's worker.
            auto attempt = m_session.BeginConnection();
            if (!attempt.has_value()) {
                ESP_LOGE(BLUFI_TAG, "Connect rejected: session closing or BLE disconnected");
                break;
            }

            SsidManager::GetInstance().AddSsid(ssid, password);
            // A connect attempt owns the radio: revoke dedicated-scan
            // ownership and release a deferred GET_WIFI_LIST with one error.
            _takeover_radio_for_connect();

            m_sta_ssid_len = static_cast<int>(std::min(ssid.size(), sizeof(m_sta_ssid)));
            memcpy(m_sta_ssid, ssid.c_str(), m_sta_ssid_len);
            memset(m_sta_bssid, 0, sizeof(m_sta_bssid));
            m_sta_conn_info = {};
            m_sta_conn_info.sta_ssid = m_sta_ssid;
            m_sta_conn_info.sta_ssid_len = m_sta_ssid_len;

            auto& wifi_manager = WifiManager::GetInstance();

            if (wifi_manager.IsInitialized()) {
                if (wifi_manager.IsConfigMode()) {
                    wifi_manager.StopConfigAp();
                }
                wifi_manager.StopStation();
                // The dedicated scan can leave esp_wifi running while no
                // station is active; WifiStation starts scanning on
                // WIFI_EVENT_STA_START, which a repeated esp_wifi_start()
                // does not re-emit. Stop the radio so StartStation begins
                // from a stopped driver and gets the start event.
                esp_wifi_stop();
            }

            if (!wifi_manager.IsInitialized() && !wifi_manager.Initialize()) {
                ESP_LOGE(BLUFI_TAG, "Failed to initialize WifiManager");
                m_session.Cancel(*attempt);
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                break;
            }

            // Context and task are set up BEFORE StartStation so a failure
            // cancels the attempt and reports without leaving an unsupervised
            // station connecting in the background.
            auto* worker_ctx = new (std::nothrow) ConnectWorkerCtx{this, *attempt, ssid};
            if (worker_ctx == nullptr) {
                ESP_LOGE(BLUFI_TAG, "Connect worker context allocation failed");
                m_session.Cancel(*attempt);
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                break;
            }
            m_active_attempt = *attempt;
            if (xTaskCreate(&Blufi::_connect_worker, "blufi_wifi_conn", 4096, worker_ctx, 5,
                            nullptr) != pdPASS) {
                ESP_LOGE(BLUFI_TAG, "Connect worker task create failed");
                delete worker_ctx;
                m_active_attempt.reset();
                m_session.Cancel(*attempt);
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                break;
            }
            // WifiStation::Stop synchronously clears the connected bit, so the
            // worker initially polls false until StartStation runs here.
            wifi_manager.StartStation();
            break;
        }
        case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
            ESP_LOGI(BLUFI_TAG, "BLUFI request wifi disconnect from AP");
            if (m_active_attempt.has_value()) {
                m_session.Cancel(*m_active_attempt);
                m_active_attempt.reset();
            }
            if (WifiManager::GetInstance().IsInitialized()) {
                WifiManager::GetInstance().StopStation();
            }
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
            auto& wifi = WifiManager::GetInstance();
            wifi_mode_t mode = GetWifiModeWithFallback(wifi);
            const int softap_conn_num = _get_softap_conn_num();

            if (wifi.IsInitialized() && wifi.IsConnected()) {
                auto current_ssid = wifi.GetSsid();
                if (!current_ssid.empty()) {
                    m_sta_ssid_len =
                        static_cast<int>(std::min(current_ssid.size(), sizeof(m_sta_ssid)));
                    memcpy(m_sta_ssid, current_ssid.c_str(), m_sta_ssid_len);
                }

                esp_blufi_extra_info_t info;
                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                memcpy(info.sta_bssid, m_sta_bssid, 6);
                info.sta_ssid = m_sta_ssid;
                info.sta_ssid_len = m_sta_ssid_len;
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, softap_conn_num,
                                                &info);
            } else if (m_session.IsConnecting()) {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONNECTING, softap_conn_num,
                                                &m_sta_conn_info);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_conn_num,
                                                &m_sta_conn_info);
            }
            ESP_LOGI(BLUFI_TAG, "BLUFI get wifi status");
            break;
        }
        case ESP_BLUFI_EVENT_RECV_STA_BSSID:
            memcpy(m_sta_config.sta.bssid, param->sta_bssid.bssid, 6);
            m_sta_config.sta.bssid_set = true;
            ESP_LOGI(BLUFI_TAG, "Recv STA BSSID");
            break;
        case ESP_BLUFI_EVENT_RECV_STA_SSID: {
            const int len = param->sta_ssid.ssid_len;
            const uint8_t* src = param->sta_ssid.ssid;
            if (!blufi_detail::BlufiValidSsidFrame(len, src)) {
                ESP_LOGE(BLUFI_TAG, "Invalid STA SSID frame (len=%d)", len);
                m_sta_config_ssid_len = 0;  // invalidate stored credential
                btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
                break;
            }
            blufi_detail::BlufiCopyCredential(m_sta_config.sta.ssid, sizeof(m_sta_config.sta.ssid),
                                              src, static_cast<size_t>(len));
            m_sta_config_ssid_len = len;
            ESP_LOGI(BLUFI_TAG, "Recv STA SSID (len=%d)", len);
            break;
        }
        case ESP_BLUFI_EVENT_RECV_STA_PASSWD: {
            const int len = param->sta_passwd.passwd_len;
            const uint8_t* src = param->sta_passwd.passwd;
            if (!blufi_detail::BlufiValidPasswordFrame(len, src)) {
                ESP_LOGE(BLUFI_TAG, "Invalid STA password frame (len=%d)", len);
                m_sta_config_passwd_len = -1;  // invalidate stored credential
                btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
                break;
            }
            blufi_detail::BlufiCopyCredential(m_sta_config.sta.password,
                                              sizeof(m_sta_config.sta.password), src,
                                              static_cast<size_t>(len));
            m_sta_config_passwd_len = len;
            ESP_LOGI(BLUFI_TAG, "Recv STA PASSWORD (len=%d)", len);
            break;
        }
        case ESP_BLUFI_EVENT_GET_WIFI_LIST:
            ESP_LOGI(BLUFI_TAG, "BLUFI get wifi list");
            _handle_get_wifi_list();
            break;
        default:
            ESP_LOGW(BLUFI_TAG, "Unhandled event: %d", event);
            break;
    }
}

void Blufi::_event_callback_trampoline(esp_blufi_cb_event_t event, esp_blufi_cb_param_t* param) {
    GetInstance()._handle_event(event, param);
}

void Blufi::_negotiate_data_handler_trampoline(uint8_t* data, int len, uint8_t** output_data,
                                               int* output_len, bool* need_free) {
    GetInstance()._dh_negotiate_data_handler(data, len, output_data, output_len, need_free);
}

int Blufi::_encrypt_func_trampoline(uint8_t iv8, uint8_t* crypt_data, int crypt_len) {
    return GetInstance()._aes_encrypt(iv8, crypt_data, crypt_len);
}

int Blufi::_decrypt_func_trampoline(uint8_t iv8, uint8_t* crypt_data, int crypt_len) {
    return GetInstance()._aes_decrypt(iv8, crypt_data, crypt_len);
}

uint16_t Blufi::_checksum_func_trampoline(uint8_t iv8, uint8_t* data, int len) {
    return _crc_checksum(iv8, data, len);
}

void Blufi::_connect_worker(void* ctx) {
    auto* worker = static_cast<ConnectWorkerCtx*>(ctx);
    Blufi* self = worker->self;
    const BlufiProvisioningSession::Attempt token = worker->token;
    auto& wifi = WifiManager::GetInstance();
    constexpr int kConnectTimeoutMs = 60000;
    constexpr TickType_t kDelayTick = pdMS_TO_TICKS(200);
    int waited_ms = 0;

    // Poll connectivity while this attempt stays the current one. A stale
    // worker aborts quietly; it never reports for a superseded attempt.
    while (waited_ms < kConnectTimeoutMs && !wifi.IsConnected() &&
           self->m_session.IsCurrent(token)) {
        vTaskDelay(kDelayTick);
        waited_ms += 200;
    }

    const bool connected = wifi.IsConnected() && self->m_session.IsCurrent(token);
    if (connected && self->m_session.BeginReport(token)) {
        // Report from locals only — the shared status fields stay owned by
        // the Bluetooth callback path (GET_WIFI_STATUS / REQ handlers), so a
        // late worker can never race another request's snapshot.
        std::string success_ssid = wifi.GetSsid();
        if (success_ssid.empty()) {
            success_ssid = worker->ssid;
        }
        uint8_t success_bssid[6] = {0};
        wifi_ap_record_t ap_info{};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            memcpy(success_bssid, ap_info.bssid, sizeof(success_bssid));
        }

        // Local snapshot only; the SDK deep-copies extra_info for its queue.
        esp_blufi_extra_info_t info = {};
        memcpy(info.sta_bssid, success_bssid, sizeof(success_bssid));
        info.sta_bssid_set = true;
        info.sta_ssid = reinterpret_cast<uint8_t*>(success_ssid.data());
        info.sta_ssid_len = static_cast<int>(success_ssid.size());
        const wifi_mode_t mode = GetWifiModeWithFallback(wifi);
        const int softap_conn_num = _get_softap_conn_num();
        const esp_err_t send_rc = esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS,
                                                                  softap_conn_num, &info);
        ESP_LOGI(BLUFI_TAG, "conn report send rc=%s", esp_err_to_name(send_rc));
        if (send_rc == ESP_OK) {
            // Keep the BLE link up briefly so the App reliably receives the
            // success report, then hand the close to deinit via the session.
            vTaskDelay(pdMS_TO_TICKS(1500));
            if (self->m_session.FinishReport(token)) {
                self->deinit();
            }
        } else {
            ESP_LOGE(BLUFI_TAG, "conn report send failed: %s", esp_err_to_name(send_rc));
            // The BLE stack may already have a deferred close waiting on this
            // attempt — release it here, never from a Bluetooth callback.
            if (self->m_session.Cancel(token)) {
                self->deinit();
            }
        }
        ESP_LOGI(BLUFI_TAG, "connected to WiFi");
    } else {
        // Timeout, abort, or a BeginReport that lost its currency: report
        // failure only while this attempt is still the live one, then cancel.
        if (self->m_session.IsCurrent(token) && self->m_session.IsBleConnected()) {
            esp_blufi_extra_info_t info = {};
            info.sta_ssid = reinterpret_cast<uint8_t*>(worker->ssid.data());
            info.sta_ssid_len = static_cast<int>(worker->ssid.size());
            const wifi_mode_t mode = GetWifiModeWithFallback(wifi);
            const int softap_conn_num = _get_softap_conn_num();
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, softap_conn_num, &info);
        }
        if (!self->m_session.Cancel(token)) {
            ESP_LOGI(BLUFI_TAG, "stale or cancelled connect worker exits");
        } else {
            ESP_LOGE(BLUFI_TAG, "Failed to connect to WiFi via esp-wifi-connect");
        }
        // BLE stays available for a retry; no teardown claimed here.
    }
    delete worker;
    vTaskDelete(nullptr);
}

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#ifdef CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
void Blufi::_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    Blufi& self = GetInstance();
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT: {
            const bool adv_data = (event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT);
            const bool success = adv_data
                                     ? (param->adv_data_cmpl.status == ESP_BT_STATUS_SUCCESS)
                                     : (param->scan_rsp_data_cmpl.status == ESP_BT_STATUS_SUCCESS);
            switch (blufi_detail::BlufiOnAdvConfigComplete(self._adv_tracker, adv_data, success)) {
                case blufi_detail::BlufiAdvAction::Start: {
                    const esp_err_t rc = esp_ble_gap_start_advertising(&self._adv_params);
                    if (rc != ESP_OK) {
                        ESP_LOGE(BLUFI_TAG, "start_advertising failed: %s", esp_err_to_name(rc));
                    }
                    break;
                }
                case blufi_detail::BlufiAdvAction::Abandon:
                    ESP_LOGE(
                        BLUFI_TAG, "adv %s config failed (status=%d); not advertising",
                        adv_data ? "data" : "scan_rsp",
                        adv_data ? param->adv_data_cmpl.status : param->scan_rsp_data_cmpl.status);
                    break;
                case blufi_detail::BlufiAdvAction::Wait:
                    break;
            }
            return;  // consumed: the SDK handler must not start advertising early
        }
        default:
            break;
    }
    esp_blufi_gap_event_handler(event, param);
}
#endif

esp_err_t Blufi::_start_mac_name_advertising() {
#ifdef CONFIG_BLUFI_INCLUDE_WIFI_MAC_IN_NAME
    uint8_t mac[6] = {0};
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(BLUFI_TAG, "Failed to read STA MAC: %s", esp_err_to_name(ret));
        return ret;
    }
    const std::string name = blufi_detail::BlufiFormatDeviceName(mac);
    ret = esp_ble_gap_set_device_name(name.c_str());
    if (ret != ESP_OK) {
        ESP_LOGE(BLUFI_TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }

    // Primary ADV keeps the SDK-compatible BluFi UUID/flags/TX-power but
    // carries no name; the full name goes to the scan response.
    static const uint8_t kBlufiServiceUuid128[16] = {
        /* 0000FFFF-0000-1000-8000-00805F9B34FB (little endian on air) */
        0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
    };
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = false;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x0006;
    adv_data.max_interval = 0x0010;
    adv_data.appearance = 0x00;
    adv_data.service_uuid_len = sizeof(kBlufiServiceUuid128);
    static uint8_t service_uuid[16];
    memcpy(service_uuid, kBlufiServiceUuid128, sizeof(service_uuid));
    adv_data.p_service_uuid = service_uuid;
    adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;

    esp_ble_adv_data_t scan_rsp = {};
    scan_rsp.set_scan_rsp = true;
    scan_rsp.include_name = true;
    scan_rsp.include_txpower = false;
    scan_rsp.manufacturer_len = 0;
    scan_rsp.p_manufacturer_data = nullptr;
    scan_rsp.service_data_len = 0;
    scan_rsp.p_service_data = nullptr;
    scan_rsp.service_uuid_len = 0;
    scan_rsp.p_service_uuid = nullptr;
    scan_rsp.flag = 0;
    // The stack builds the Complete Local Name field from the device name set
    // above; include_name picks it up. Name length already <= 31 - 2.

    _adv_params = {};
    _adv_params.adv_int_min = 0x20;
    _adv_params.adv_int_max = 0x40;
    _adv_params.adv_type = ADV_TYPE_IND;
    _adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    _adv_params.channel_map = ADV_CHNL_ALL;
    _adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    _adv_tracker = {};
    _adv_tracker.pending = true;

    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        _adv_tracker.pending = false;
        ESP_LOGE(BLUFI_TAG, "adv data config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = esp_ble_gap_config_adv_data(&scan_rsp);
    if (ret != ESP_OK) {
        _adv_tracker.pending = false;
        ESP_LOGE(BLUFI_TAG, "scan rsp config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Advertising starts when both config-complete events arrive.
    return ESP_OK;
#else
    esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
    esp_blufi_adv_start();
    return ESP_OK;
#endif
}
#endif /* CONFIG_BT_BLUEDROID_ENABLED */
