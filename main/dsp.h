/*
 * dsp.h — IQ processing: WBFM demod (→ audio) + FFT spectrum (→ UI).
 *
 * Owns the sole ring consumer task. Rendering lives in ui.c, which reads the
 * spectrum snapshot exposed here.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start the IQ-processing task. Call after sdr_stream_start(). */
void dsp_start(void);

/* Demodulation modes. */
#define DSP_MODE_WFM 0 /* wideband FM (broadcast)                            */
#define DSP_MODE_NFM 1 /* narrowband FM (NOAA / public safety voice)         */
#define DSP_MODE_AM  2 /* amplitude modulation (airband 118-137 MHz)         */
#define DSP_MODE_COUNT 3

/* Select the demodulator. Changes the decimation chain + audio shaping. */
void dsp_set_mode(int mode);
int dsp_get_mode(void);

/* Number of spectrum bins exposed (fftshifted: index 0 = lowest freq). */
int dsp_spectrum_bins(void);

/* Pointer to the latest spectrum frame (NOMINAL dBm per bin), or NULL if none
 * yet. Valid until the next frame; read-only.
 *
 * The values already carry the dBm offset (see SPECTRUM_DBM_OFFSET in dsp.c), so
 * callers can plot or label them directly. They are not calibrated: use them for
 * relative comparison, not absolute level measurement.
 *
 * Note this is a DIFFERENT scale from dsp_signal_db(), which is demodulated IF
 * power in raw dB. Do not mix the two. */
const float *dsp_spectrum(void);

/* Squelch: mute audio when the signal level falls below `db`. Use a very low
 * value (e.g. -200) to disable. */
void dsp_set_squelch_db(float db);
float dsp_get_squelch_db(void);

/* Latch the demodulated audio off/on regardless of signal level. Independent of
 * the squelch: this is a user request, the squelch is an automatic gate. */
void dsp_set_mute(bool mute);
bool dsp_get_mute(void);

/* Current smoothed signal level (dB, same scale as the squelch threshold).
 * This is demodulated IF power, NOT the dsp_spectrum() dBm scale. */
float dsp_signal_db(void);

/* Drop the in-progress spectrum accumulation (e.g. after a retune) so the next
 * frame contains only data from the NEW frequency. Also makes dsp_spectrum()
 * return NULL until the next snapshot lands. */
void dsp_flush_spectrum(void);

/* Tracked noise floor (dB, same scale as dsp_signal_db): falls fast, rises
 * slowly — used by the auto-scan to judge occupancy without a squelch set. */
float dsp_noise_floor_db(void);

#ifdef __cplusplus
}
#endif
