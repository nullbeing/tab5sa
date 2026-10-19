/*
 * shot.h — host-triggered screen capture over the USB-Serial-JTAG port.
 *
 * The Tab5's USB-C port is the same link used for flashing and logging, so a
 * screenshot needs no extra wiring: the host sends one request byte and the
 * firmware streams the live panel frame back.
 *
 * The frame sent is the real 720x1280 panel framebuffer (RGB565), not the
 * 640x360 landscape buffer the UI draws into — that is what "screenshot" means
 * here, and it keeps the PPA's scale/rotate in the loop so what you get is
 * literally what the panel showed.
 *
 * Console logging shares this link, so a log line can land in the middle of a
 * transfer. The header therefore carries a checksum and the host retries on
 * mismatch rather than saving a torn image.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Request byte the host sends to ask for a frame. */
#define SHOT_REQ 'S'

/* Panel geometry that gets captured (portrait, post-PPA). */
#define SHOT_W 720
#define SHOT_H 1280

/* Magic at the start of every frame header, so the host can resynchronise if it
 * attached mid-stream instead of guessing from byte counts. */
#define SHOT_MAGIC 0x35415354u /* "TSA5" little-endian */

/* Wire format, little-endian, 16 bytes, then `bytes` of raw RGB565.
 * Fields are sent one at a time rather than as a struct: the ESP32-P4 and the
 * host do not have to agree on padding. `sum` is a 32-bit additive checksum of
 * the pixel payload. */
typedef struct {
    uint32_t magic;
    uint16_t w;
    uint16_t h;
    uint32_t bytes;
    uint32_t sum;
} shot_hdr_t;

/* Install the driver and start the capture task. Call after ui_start(), since
 * it reads the panel framebuffer that ui.c owns. Non-fatal on failure: without
 * it the firmware simply cannot be screenshotted. */
void shot_start(void);

#ifdef __cplusplus
}
#endif
