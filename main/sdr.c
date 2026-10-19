/*
 * sdr.c — RTL-SDR device management, IQ streaming, and a PSRAM ring buffer.
 *
 * IMPORTANT: on the Tab5 the dongle shares the single USB port with the
 * flashing/serial link, so most of the time there is NO dongle attached. This
 * file is therefore built as a supervisor, not a one-shot bring-up:
 *
 *   supervisor task (core 1)
 *     ├─ no dongle: poll enumeration every second, forever. Never blocks
 *     │             callers, never spins, never touches NULL handles.
 *     └─ dongle:    configure + run rtlsdr_read_async() on its own task.
 *                   If the stream ends (unplugged, stall, error) the
 *                   supervisor tears it down and goes back to searching.
 *
 * The UI/DSP side polls sdr_is_present() and sdr_read(), both of which are safe
 * to call at any time — they simply report "nothing" when no dongle is there.
 */
#include "sdr.h"

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "rtl-sdr.h"
#include "libusb.h" /* usbhost_begin() */

static const char *TAG = "sdr";

/* USB transfer geometry handed to rtlsdr_read_async(). buf_len must be a
 * multiple of 64 (USB packet size); 16 KiB ≈ 8 ms at 1.152 Msps. */
#define SDR_XFER_NUM 8
#define SDR_XFER_LEN 16384

/* ~0.5 s of IQ at 1.152 Msps. Power of two so we can mask instead of modulo. */
#define RING_SIZE (1u << 20) /* 1 MiB */
#define RING_MASK (RING_SIZE - 1)

#define RETRY_MS      1000 /* re-enumeration period while no dongle   */
#define TEARDOWN_MS   500  /* settle time after a dongle disappears   */

/* ---- single-producer / single-consumer byte ring in PSRAM ---------------- */
static uint8_t *s_ring;
static volatile uint32_t s_head; /* producer write offset (free-running) */
static volatile uint32_t s_tail; /* consumer read offset  (free-running) */

static rtlsdr_dev_t *s_dev;
static volatile uint64_t s_produced;
static volatile uint64_t s_dropped;
static uint32_t s_sample_rate = SDR_DEFAULT_SAMPLE_RATE;
static uint32_t s_center_freq = SDR_DEFAULT_CENTER_FREQ;
static const char *s_tuner = "none";

/* Device state, owned by the supervisor task. */
static volatile bool s_present;      /* dongle open + streaming          */
static volatile bool s_stream_ended; /* read_async returned              */
static TaskHandle_t s_stream_task;

static const char *tuner_name(enum rtlsdr_tuner t)
{
    switch (t) {
    case RTLSDR_TUNER_E4000:  return "Elonics E4000";
    case RTLSDR_TUNER_FC0012: return "Fitipower FC0012";
    case RTLSDR_TUNER_FC0013: return "Fitipower FC0013";
    case RTLSDR_TUNER_FC2580: return "FCI FC2580";
    case RTLSDR_TUNER_R820T:  return "Rafael Micro R820T";
    case RTLSDR_TUNER_R828D:  return "Rafael Micro R828D";
    default:                  return "unknown";
    }
}

/* librtlsdr async callback — runs on the streaming task. Push IQ into the ring,
 * dropping whole buffers if the consumer has fallen behind. */
static void on_iq(unsigned char *buf, uint32_t len, void *ctx)
{
    (void)ctx;
    if (!s_ring || !buf || !len) return;

    uint32_t head = s_head;
    uint32_t space = RING_SIZE - (head - s_tail);
    if (len > space) {
        s_dropped += len;
        return;
    }
    uint32_t first = RING_SIZE - (head & RING_MASK);
    if (first >= len) {
        memcpy(s_ring + (head & RING_MASK), buf, len);
    } else {
        memcpy(s_ring + (head & RING_MASK), buf, first);
        memcpy(s_ring, buf + first, len - first);
    }
    s_head = head + len;      /* publish after the data is in place */
    s_produced += len;
}

static void stream_task(void *arg)
{
    (void)arg;
    /* rtlsdr_read_async blocks here pumping USB transfers until cancelled. */
    int r = rtlsdr_read_async(s_dev, on_iq, NULL, SDR_XFER_NUM, SDR_XFER_LEN);
    ESP_LOGW(TAG, "rtlsdr_read_async returned %d (stream ended)", r);
    s_stream_ended = true; /* hand control back to the supervisor */
    s_stream_task = NULL;
    vTaskDelete(NULL);
}

/* Apply tuning to an open device. Returns false if the tuner rejects it. */
static bool configure(rtlsdr_dev_t *dev)
{
    if (rtlsdr_set_sample_rate(dev, s_sample_rate) < 0)
        ESP_LOGW(TAG, "set_sample_rate failed");
    if (rtlsdr_set_center_freq(dev, s_center_freq) < 0) {
        ESP_LOGW(TAG, "set_center_freq failed");
        return false;
    }
    rtlsdr_set_tuner_gain_mode(dev, 0); /* automatic gain */
    if (rtlsdr_reset_buffer(dev) < 0)   /* mandatory before reading */
        ESP_LOGW(TAG, "reset_buffer failed");
    return true;
}

/* Try to open a dongle and start streaming. Returns true on success. */
static bool try_start(void)
{
    uint32_t count = rtlsdr_get_device_count();
    if (count == 0) return false; /* the common case: nothing plugged in */

    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, 0) != 0 || !dev) {
        ESP_LOGW(TAG, "rtlsdr_open failed (count=%" PRIu32 ")", count);
        return false;
    }

    if (!configure(dev)) {
        ESP_LOGW(TAG, "tuner rejected configuration; closing");
        rtlsdr_close(dev);
        return false;
    }

    s_dev = dev;
    s_tuner = tuner_name(rtlsdr_get_tuner_type(dev));
    s_head = s_tail = 0;
    s_produced = s_dropped = 0;
    s_stream_ended = false;

    BaseType_t ok = xTaskCreatePinnedToCore(stream_task, "sdr_stream", 8192,
                                            NULL, 12, &s_stream_task, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create stream task");
        s_stream_task = NULL;
        rtlsdr_close(dev);
        s_dev = NULL;
        return false;
    }

    s_present = true;
    ESP_LOGI(TAG, "streaming: %.3f MHz @ %" PRIu32 " Hz — tuner: %s",
             s_center_freq / 1e6, s_sample_rate, s_tuner);
    return true;
}

/* Stop streaming and release the device. Safe to call in any state.
 *
 * rtlsdr_close() busy-waits (unbounded) for the async engine to reach
 * RTLSDR_INACTIVE, so we must NOT call it while the stream task may still be
 * alive — that would hang the supervisor forever. If the task refuses to exit
 * we keep s_dev set and simply retry on the next supervisor pass. */
static void teardown(void)
{
    s_present = false;
    if (!s_dev) return;

    rtlsdr_cancel_async(s_dev); /* ask read_async() to unwind */

    for (int i = 0; i < 200 && s_stream_task; i++) /* <= 2 s */
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_stream_task) {
        ESP_LOGE(TAG, "stream task still running; deferring close");
        return; /* s_dev stays set: retried next pass, never close-under-use */
    }

    rtlsdr_close(s_dev);
    s_dev = NULL;
    s_tuner = "none";
    s_head = s_tail = 0; /* drop stale IQ so the UI sees "no data" */
    ESP_LOGI(TAG, "dongle released; searching again");
}

static void supervisor_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_dev) {
            /* Open but not streaming: stream ended, or a previous teardown
             * could not finish. Keep trying to release it. */
            if (s_stream_ended || !s_present) {
                teardown();
                vTaskDelay(pdMS_TO_TICKS(TEARDOWN_MS));
            }
        } else {
            try_start(); /* no-op (and cheap) when nothing is plugged in */
        }
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
}

void sdr_start(uint32_t center_freq, uint32_t sample_rate)
{
    s_center_freq = center_freq;
    s_sample_rate = sample_rate;

    s_ring = heap_caps_malloc(RING_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_ring) {
        /* Without a ring there is nothing to stream into. The UI still runs
         * and shows "no data" — never abort the whole application. */
        ESP_LOGE(TAG, "failed to allocate %u-byte ring in PSRAM", RING_SIZE);
    }

    usbhost_begin(); /* installs the host stack; harmless without a dongle */

    BaseType_t ok = xTaskCreatePinnedToCore(supervisor_task, "sdr_sup", 6144,
                                            NULL, 11, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create supervisor task");
    }
}

bool sdr_is_present(void) { return s_present; }

int sdr_set_center_freq(uint32_t freq)
{
    s_center_freq = freq; /* remembered even with no dongle attached */
    if (!s_dev || !s_present) return 0;
    return (rtlsdr_set_center_freq(s_dev, freq) < 0) ? -1 : 0;
}

void sdr_get_stats(sdr_stats_t *out)
{
    if (!out) return;
    out->produced_bytes = s_produced;
    out->dropped_bytes = s_dropped;
    out->sample_rate = s_sample_rate;
    out->center_freq = s_center_freq;
    out->tuner = s_tuner;
    out->present = s_present;
}

size_t sdr_read(uint8_t *dst, size_t max)
{
    if (!s_ring || !dst || !max) return 0;

    uint32_t tail = s_tail;
    uint32_t avail = s_head - tail;
    if (avail == 0) return 0;
    if (max > avail) max = avail;

    uint32_t first = RING_SIZE - (tail & RING_MASK);
    if (first >= max) {
        memcpy(dst, s_ring + (tail & RING_MASK), max);
    } else {
        memcpy(dst, s_ring + (tail & RING_MASK), first);
        memcpy(dst + first, s_ring, max - first);
    }
    s_tail = tail + max;
    return max;
}
