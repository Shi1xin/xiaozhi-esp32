"""Host test for main/settings.cc erase/commit semantics.

The REAL settings.cc is compiled against stub ESP-IDF headers; a fake NVS
store counts commits/erases so destructor-flush behavior is observable.
"""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
SETTINGS_CC = ROOT / "main/settings.cc"
SETTINGS_H = ROOT / "main/settings.h"

NVS_FLASH_H = r"""
#ifndef NVS_FLASH_STUB_H
#define NVS_FLASH_STUB_H
#include <stddef.h>
typedef int esp_err_t;
typedef unsigned int nvs_handle_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NVS_NOT_FOUND 0x1106
#define NVS_READONLY 0
#define NVS_READWRITE 1
#define ESP_ERROR_CHECK(x) do { esp_err_t _e=(x); if(_e!=ESP_OK) for(;;){} } while(0)
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t nvs_open(const char*, int, nvs_handle_t*);
esp_err_t nvs_get_str(nvs_handle_t, const char*, char*, size_t*);
esp_err_t nvs_set_str(nvs_handle_t, const char*, const char*);
esp_err_t nvs_get_i32(nvs_handle_t, const char*, int*);
esp_err_t nvs_set_i32(nvs_handle_t, const char*, int);
esp_err_t nvs_get_u8(nvs_handle_t, const char*, unsigned char*);
esp_err_t nvs_set_u8(nvs_handle_t, const char*, unsigned char);
esp_err_t nvs_erase_key(nvs_handle_t, const char*);
esp_err_t nvs_erase_all(nvs_handle_t);
esp_err_t nvs_commit(nvs_handle_t);
void nvs_close(nvs_handle_t);
#ifdef __cplusplus
}
#endif
#endif
"""

ESP_LOG_H = r"""
#ifndef ESP_LOG_STUB_H
#define ESP_LOG_STUB_H
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#endif
"""

NVS_IMPL_C = r"""
#include "nvs_flash.h"
#include <string.h>
// Fake NVS store: keys present/absent per handle; counters expose behavior.
static char g_keys[64][64];
static int g_nkeys = 0;
static int g_handles = 0;
int g_commit_calls = 0;
int g_erase_key_calls = 0;
int g_erase_all_calls = 0;
int g_close_calls = 0;
int g_missing_key_return = ESP_ERR_NVS_NOT_FOUND;

static int key_index(const char* key) {
    for (int i = 0; i < g_nkeys; i++) if (strcmp(g_keys[i], key) == 0) return i;
    return -1;
}
void fake_nvs_seed(const char* key) {
    strncpy(g_keys[g_nkeys++], key, sizeof(g_keys[0]) - 1);
}
void fake_nvs_reset() {
    g_nkeys = 0;
    g_commit_calls = g_erase_key_calls = g_erase_all_calls = g_close_calls = 0;
}
esp_err_t nvs_open(const char*, int mode, nvs_handle_t* h) {
    *h = (nvs_handle_t)(++g_handles + (mode == NVS_READWRITE ? 1000 : 0));
    return ESP_OK;
}
esp_err_t nvs_get_str(nvs_handle_t, const char* k, char* out, size_t* len) {
    if (out == NULL) { *len = 4; return ESP_OK; }
    strcpy(out, "abc"); return ESP_OK;
}
esp_err_t nvs_set_str(nvs_handle_t, const char* k, const char*) {
    if (key_index(k) < 0) fake_nvs_seed(k); return ESP_OK;
}
esp_err_t nvs_get_i32(nvs_handle_t, const char*, int* v) { *v = 1; return ESP_OK; }
esp_err_t nvs_set_i32(nvs_handle_t, const char* k, int) {
    if (key_index(k) < 0) fake_nvs_seed(k); return ESP_OK;
}
esp_err_t nvs_get_u8(nvs_handle_t, const char*, unsigned char* v) { *v = 1; return ESP_OK; }
esp_err_t nvs_set_u8(nvs_handle_t, const char* k, unsigned char) {
    if (key_index(k) < 0) fake_nvs_seed(k); return ESP_OK;
}
esp_err_t nvs_erase_key(nvs_handle_t, const char* k) {
    g_erase_key_calls++;
    if (key_index(k) < 0) return g_missing_key_return;
    int i = key_index(k);
    memmove(g_keys[i], g_keys[i+1], (g_nkeys-i-1)*64);
    g_nkeys--;
    return ESP_OK;
}
esp_err_t nvs_erase_all(nvs_handle_t) { g_erase_all_calls++; g_nkeys = 0; return ESP_OK; }
esp_err_t nvs_commit(nvs_handle_t) { g_commit_calls++; return ESP_OK; }
void nvs_close(nvs_handle_t) { g_close_calls++; }
"""

TEST_MAIN = r"""
#include <cassert>
#include <cstdio>
#include "settings.h"

extern "C" {
extern int g_commit_calls, g_erase_key_calls, g_erase_all_calls, g_close_calls;
extern void fake_nvs_seed(const char*);
extern void fake_nvs_reset();
}

int main() {
    // erase-existing commits at destruction
    fake_nvs_reset(); fake_nvs_seed("k");
    { Settings s("ns", true); s.EraseKey("k"); }
    assert(g_erase_key_calls == 1 && g_commit_calls == 1 && g_close_calls == 1);

    // erase-missing: NOT_FOUND is a no-op, no commit
    fake_nvs_reset();
    { Settings s("ns", true); s.EraseKey("missing"); }
    assert(g_erase_key_calls == 1 && g_commit_calls == 0);

    // erase-all commits
    fake_nvs_reset(); fake_nvs_seed("a");
    { Settings s("ns", true); s.EraseAll(); }
    assert(g_erase_all_calls == 1 && g_commit_calls == 1);

    // read-only erase: no mutation, no commit
    fake_nvs_reset(); fake_nvs_seed("k");
    { Settings s("ns", false); s.EraseKey("k"); s.EraseAll(); }
    assert(g_erase_key_calls == 0 && g_erase_all_calls == 0 && g_commit_calls == 0);

    // no-change write scope: no commit
    fake_nvs_reset();
    { Settings s("ns", true); (void)s.GetInt("x", 0); }
    assert(g_commit_calls == 0 && g_close_calls == 1);

    // baseline: a real set still commits (unchanged behavior)
    fake_nvs_reset();
    { Settings s("ns", true); s.SetInt("n", 3); }
    assert(g_commit_calls == 1);

    printf("settings-ok\n");
    return 0;
}
"""


@unittest.skipUnless(shutil.which("c++"), "requires a host C++ compiler")
class SettingsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        tmp = pathlib.Path(cls.tmp.name)
        (tmp / "nvs_flash.h").write_text(NVS_FLASH_H)
        (tmp / "esp_log.h").write_text(ESP_LOG_H)
        (tmp / "nvs_impl.c").write_text(NVS_IMPL_C)
        (tmp / "settings.h").write_text(SETTINGS_H.read_text())
        (tmp / "test_main.cc").write_text(TEST_MAIN)
        nvs_obj = tmp / "nvs_impl.o"
        subprocess.run(["cc", "-std=c99", "-I", str(tmp), "-c",
                        str(tmp / "nvs_impl.c"), "-o", str(nvs_obj)],
                       check=True, capture_output=True, text=True)
        cls.binary = str(tmp / "settings_test")
        subprocess.run(
            ["c++", "-std=c++17", "-Wall", "-Werror", "-I", str(tmp),
             "-I", str(ROOT / "main"),
             str(SETTINGS_CC), str(tmp / "test_main.cc"), str(nvs_obj),
             "-o", cls.binary],
            check=True, capture_output=True, text=True)

    def test_erase_commit_binary(self):
        result = subprocess.run([self.binary], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("settings-ok", result.stdout)


if __name__ == "__main__":
    unittest.main()
