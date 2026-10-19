/*
 * shot.c — stream the live panel frame to the host on request.
 *
 * The request comes in on the USB-Serial-JTAG link (the same USB-C port used
 * for flashing and logs), so a screenshot needs no extra wiring. See shot.h for
 * the wire format.
 *
 * Two things make this harder than it looks:
 *
 *  1. Tearing. The UI double-buffers through the PPA: present() writes one
 *     buffer, flushes it, then flips. Reading the buffer the PPA is currently
 *     writing would give a half-updated frame. ui_panel_snapshot() copies the
 *     most recently *presented* buffer under the same lock present() takes, so
 *     the copy is always a whole frame.
 *
 *  2. Interference. Console logging shares this link, and the heartbeat logs
 *     every 5 s. A log line can therefore land in the middle of a transfer. The
 *     header carries a checksum and the host retries on mismatch; nothing here
 *     has to suppress logging, which would mean touching every other module.
 */
#include "shot.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "ui.h"

static const char *TAG = "shot";

/* One panel frame, held so the USB write does not race the PPA. Allocated in
 * PSRAM: 1.8 MB is far too much for internal RAM next to the IQ ring. */
static uint16_t *s_snap;

static uint32_t sum32(const uint8_t *p, size_t n)
{
    /* Additive over 32-bit words: cheap, order-independent enough to catch a log
     * line spliced into the payload, and requires no state on the host side. */
    uint32_t s = 0;
    size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        s += (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
             ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
    }
    for (; i < n; i++) s += p[i];
    return s;
}

/* Write the whole buffer or give up. Bounded retries: a host that stops reading
 * must not wedge this task forever.
 *
 * Must be chunked. usb_serial_jtag_write_bytes() pushes into a FreeRTOS ring
 * buffer of tx_buffer_size bytes, and for a BYTEBUF ring the send is
 * all-or-nothing: handing it a 1.8 MB frame when the ring holds 8 KB fails
 * every time and returns 0. Chunking also lets each write block until the ISR
 * has drained space, which paces the transfer to the USB link. */
static bool send_all(const void *src, size_t n)
{
    const uint8_t *p = src;
    const size_t CHUNK = 2048;
    while (n) {
        size_t want = n < CHUNK ? n : CHUNK;
        int w = usb_serial_jtag_write_bytes(p, want, pdMS_TO_TICKS(1000));
        if (w <= 0) {
            /* Timeout: the host is not draining. Drop this frame rather than
             * spin; the host will simply ask again. */
            return false;
        }
        p += w;
        n -= (size_t)w;
    }
    return true;
}

static void put16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)v;
    d[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *d, uint32_t v)
{
    d[0] = (uint8_t)v;
    d[1] = (uint8_t)(v >> 8);
    d[2] = (uint8_t)(v >> 16);
    d[3] = (uint8_t)(v >> 24);
}

static void shot_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t c;
        int n = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(200));
        if (n != 1 || c != SHOT_REQ) continue;

        int w = 0, h = 0;
        if (!ui_panel_snapshot(s_snap, &w, &h)) {
            ESP_LOGW(TAG, "no frame presented yet");
            continue;
        }

        const size_t bytes = (size_t)w * (size_t)h * 2;

        /* Header built byte by byte: the host must not have to match this
         * compiler's struct padding. */
        uint8_t hdr[16];
        put32(hdr + 0, SHOT_MAGIC);
        put16(hdr + 4, (uint16_t)w);
        put16(hdr + 6, (uint16_t)h);
        put32(hdr + 8, (uint32_t)bytes);
        put32(hdr + 12, sum32((const uint8_t *)s_snap, bytes));

        if (send_all(hdr, sizeof hdr) && send_all(s_snap, bytes))
            ESP_LOGI(TAG, "sent %dx%d, %u bytes", w, h, (unsigned)bytes);
        else
            ESP_LOGW(TAG, "transfer aborted (host not reading)");
    }
}

void shot_start(void)
{
    s_snap = heap_caps_malloc((size_t)SHOT_W * SHOT_H * 2, MALLOC_CAP_SPIRAM);
    if (!s_snap) {
        ESP_LOGE(TAG, "no PSRAM for the capture buffer; screenshots disabled");
        return;
    }

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 8192;
    cfg.rx_buffer_size = 1024;
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag driver install failed: %s",
                 esp_err_to_name(err));
        return;
    }

    /* The console was writing straight to the peripheral. Once the driver owns
     * the port, console output has to go through it as well, or log lines and
     * frame data would be written by two different paths and interleave into
     * garbage. */
    usb_serial_jtag_vfs_use_driver();

    if (xTaskCreatePinnedToCore(shot_task, "shot", 4096, NULL, 3, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "failed to create capture task");
        return;
    }
    ESP_LOGI(TAG, "screenshot ready: send '%c' on this port", SHOT_REQ);
}
