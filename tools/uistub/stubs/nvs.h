/*
 * nvs_flash.h / nvs.h stub for the host preview.
 *
 * ui.c stores the favorites list in NVS. On the host there is nothing to
 * persist between runs, so init always succeeds and the blob is kept in a small
 * static buffer — enough to exercise save/load/erase paths without a flash
 * partition. NVS_ERR is deliberately not always ESP_OK: one code path in
 * fav_load() depends on a failed read being handled, and the preview should
 * compile that path too.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL (-1)
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110D
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1110

typedef uint32_t nvs_handle_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1

#define NVS_STUB_MAX 256

static struct {
    uint8_t data[NVS_STUB_MAX];
    size_t len;
    int valid;
} g_nvs_stub;

static inline esp_err_t nvs_flash_init(void) { return ESP_OK; }
static inline esp_err_t nvs_flash_erase(void) { g_nvs_stub.valid = 0; return ESP_OK; }

static inline esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *out)
{
    (void)ns;
    (void)mode;
    if (!out) return ESP_FAIL;
    *out = 1;
    return ESP_OK;
}

static inline void nvs_close(nvs_handle_t h) { (void)h; }
static inline esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }

static inline esp_err_t nvs_set_blob(nvs_handle_t h, const char *key,
                                     const void *val, size_t len)
{
    (void)h;
    (void)key;
    if (len > NVS_STUB_MAX) return ESP_FAIL;
    memcpy(g_nvs_stub.data, val, len);
    g_nvs_stub.len = len;
    g_nvs_stub.valid = 1;
    return ESP_OK;
}

static inline esp_err_t nvs_get_blob(nvs_handle_t h, const char *key,
                                     void *out, size_t *len)
{
    (void)h;
    (void)key;
    if (!g_nvs_stub.valid) return ESP_FAIL;
    if (!len || *len < g_nvs_stub.len) return ESP_FAIL;
    memcpy(out, g_nvs_stub.data, g_nvs_stub.len);
    *len = g_nvs_stub.len;
    return ESP_OK;
}
