/*
 * sdr.h — RTL-SDR device management, IQ streaming, and a PSRAM ring buffer.
 *
 * The dongle shares the Tab5's single USB port with the flashing/serial link,
 * so "no dongle present" is the NORMAL state during development. Everything
 * here is therefore built around a background supervisor task: it retries
 * enumeration forever, streams when a dongle appears, and tears the stream
 * down and goes back to searching when it is unplugged. Callers never block
 * on device presence.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default radio configuration (FM broadcast band). 1.152 Msps decimates cleanly
 * by 4 then 6 to 48 kHz audio (1.152M/4 = 288k, /6 = 48k). */
#define SDR_DEFAULT_SAMPLE_RATE 1152000u
#define SDR_DEFAULT_CENTER_FREQ 100000000u /* 100.0 MHz */

typedef struct {
    uint64_t produced_bytes; /* total IQ bytes pushed by the USB callback   */
    uint64_t dropped_bytes;  /* bytes discarded because the ring was full   */
    uint32_t sample_rate;    /* configured RTL2832 sample rate (Hz)         */
    uint32_t center_freq;    /* requested tuner centre frequency (Hz)       */
    const char *tuner;       /* tuner name string, "none" when absent       */
    bool present;            /* a dongle is open and IQ is streaming        */
} sdr_stats_t;

/* Bring up the USB host and start the supervisor task. Non-blocking: returns
 * immediately whether or not a dongle is plugged in. `center_freq` is the
 * frequency to tune to, and is re-applied automatically after a hot-plug. */
void sdr_start(uint32_t center_freq, uint32_t sample_rate);

/* True while a dongle is open and streaming. Safe to poll from any task. */
bool sdr_is_present(void);

/* Retune. Always records the request (so it survives a hot-plug) and only
 * touches hardware when a dongle is actually streaming. Returns 0 if the
 * request was accepted, -1 if the tuner rejected it. */
int sdr_set_center_freq(uint32_t freq);

/* Snapshot the current streaming statistics. */
void sdr_get_stats(sdr_stats_t *out);

/* Consume up to `max` IQ bytes from the ring buffer into `dst`.
 * Returns the number of bytes copied (0 if empty or no dongle). */
size_t sdr_read(uint8_t *dst, size_t max);

#ifdef __cplusplus
}
#endif
