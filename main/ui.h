/*
 * ui.h — instrument-style spectrum console (display only, no touch).
 *
 * Renders a dense SDR-analyzer face into a 640x360 landscape buffer and uses
 * the P4 PPA to scale x2 + rotate 90° onto the 720x1280 portrait panel:
 * status header with a 7-segment frequency readout, green filled spectrum with
 * a dBm graticule, frequency ruler, and a rainbow waterfall with a time axis.
 *
 * Tuning is not wired to any input device in this build; callers (a future
 * encoder, keypad, or serial console) drive it through ui_set_freq() /
 * ui_set_mode() / ui_set_volume().
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the render task. Call after sdr_stream_start() + dsp_start(). */
void ui_start(void);

/* Retune the receiver (clamped to the tuner range) and refresh the display. */
void ui_set_freq(uint32_t hz);
uint32_t ui_get_freq(void);

/* Demod mode: 0 = wideband FM, 1 = narrowband FM, 2 = AM. */
void ui_set_mode(int mode);

/* Speaker volume, 0..100. */
void ui_set_volume(int vol);

/* Copy the most recently presented panel frame (PANEL_W x PANEL_H, RGB565)
 * into `dst`, which must hold at least 720*1280*2 bytes. Returns false before
 * the first frame has been presented.
 *
 * Safe against tearing: it takes the same lock present() holds while the PPA is
 * writing a buffer, so the copy is always a whole frame rather than a half-
 * updated one. Used by the screenshot channel (shot.c). */
#include <stdbool.h>
bool ui_panel_snapshot(uint16_t *dst, int *w, int *h);

#ifdef __cplusplus
}
#endif
