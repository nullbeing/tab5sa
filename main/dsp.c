/*
 * dsp.c — IQ processing: WBFM demodulation (→ audio) + FFT spectrum (→ ui.c).
 *
 * iq_proc (core 1, high prio) is the SOLE ring consumer. Every block:
 *   - FM demod chain → audio_play()   (must be gap-free)
 *   - signal-level estimate for squelch
 * Every FFT_EVERY blocks it also runs one FFT, snapshotting a dB-per-bin frame
 * into a double buffer that ui.c reads. Rendering itself is in ui.c.
 *
 * Demod @ 1.152 MS/s IQ:  /4 boxcar → 288 kHz → polar discriminator
 *                         → /6 boxcar → 48 kHz → 75 µs de-emphasis → int16.
 */
#include "dsp.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dsp.h"

#include "sdr.h"
#include "audio.h"

static const char *TAG = "dsp";

#define FFT_N 1024

#define FFT_EVERY     8
#define FFT_PER_FRAME 6

/*
 * Nominal dBm offset, applied when a frame is published.
 *
 * The FFT yields 10*log10(|X|^2) of windowed 8-bit IQ: an arbitrary scale
 * (roughly 58..90) with no absolute reference. This constant shifts it onto an
 * axis that reads like a spectrum analyzer.
 *
 * It is NOT a calibration. Nothing here accounts for the dongle's gain setting,
 * tuner conversion loss, IF filter shape or the window's coherent gain, so the
 * numbers are good for comparing signals against each other and against the
 * noise floor — not for measuring absolute level.
 *
 * It belongs here, beside the FFT that produces the scale, rather than in ui.c:
 * mapping "FFT magnitude" to "level" is a property of the DSP chain. Change
 * FFT_N, the window or the averaging and this is the one place that has to be
 * revisited — previously ui.c held it, so such a change would have silently
 * shifted every reading on screen with nothing to catch it.
 */
#define SPECTRUM_DBM_OFFSET (-168.0f)

/* Demod parameters per mode, applied at 1.152 MS/s IQ:
 *   WFM (broadcast): /4 → 288 kHz IF, /6 → 48 kHz audio, 75 µs de-emphasis.
 *   NFM (voice):     /24 → 48 kHz IF, no further decim, no de-emphasis, more gain.
 *   AM (airband):    /24 → 48 kHz IF, envelope detector, no de-emphasis.
 */
static volatile int s_mode;
static int s_decim_if = 4, s_decim_audio = 6;
static float s_inv_if = 1.0f / 4, s_inv_audio = 1.0f / 6;
static float s_deemph_a = 0.24f;
static float s_gain = 9000.0f;

static float s_window[FFT_N];
static float s_fft[2 * FFT_N];
static float s_acc[FFT_N];
static uint32_t s_acc_cnt;
static uint8_t s_blk[2 * FFT_N];

/* Double-buffered dB-per-bin spectrum (fftshifted) for the UI (lock-free). */
static float s_spec[2][FFT_N];
static volatile int s_ready = -1;

/* FM demod running state. */
static float d_accI, d_accQ;
static int d_cntIF;
static float d_pI, d_pQ;
static float d_accAud;
static int d_cntAud;
static float d_deemph;

/* AM DC tracker: the carrier is a large constant term in the envelope, so it
 * has to be removed or the audio is offset by a DC step (audible as a thump and
 * as a loss of headroom). A one-pole high-pass follows the carrier level
 * automatically, which also tracks AGC changes on the dongle. */
static float d_dc;
#define AM_DC_ALPHA 0.003f /* ≈23 Hz high-pass at the 48 kHz IF rate */

/* Squelch / signal level. */
static volatile float s_sig_db = -120.0f;
static volatile float s_squelch_db = -200.0f; /* disabled by default */
/* Master mute for the demodulated output, driven by the play/stop button. This
 * is deliberately separate from the squelch: the squelch closes per-block when
 * the signal drops (so it chatters at the edge of a carrier), while this is a
 * latch the user owns and that must not depend on the signal at all. */
static volatile bool s_mute;

/* Noise-floor tracker (same dB scale): follows new minima instantly, rises
 * only slowly so occupied channels don't lift the floor. Seeded from the first
 * real measurement — starting at -120 would leave the auto-scan's occupancy
 * threshold far too low until the slow rise caught up. */
static volatile float s_nf_db = -120.0f;
static bool s_nf_seeded;

/* Cross-core spectrum flush: ui.c sets the flag after a retune; the iq_proc
 * task (the only writer of s_acc) performs the clear itself at a safe point. */
static volatile bool s_flush_req;

static void demod_block(void)
{
    int16_t aud[FFT_N + 4]; /* worst case (NFM /24, audio /1): ~43/block */
    int naud = 0;
    float ifpow = 0;
    int nif = 0;
    bool open = !s_mute && s_sig_db >= s_squelch_db;

    int dif = s_decim_if, daud = s_decim_audio;
    float inv_if = s_inv_if, inv_aud = s_inv_audio, dea = s_deemph_a, gain = s_gain;
    bool am = (s_mode == DSP_MODE_AM);

    for (int i = 0; i < FFT_N; i++) {
        float I = (float)s_blk[2 * i] - 127.4f;
        float Q = (float)s_blk[2 * i + 1] - 127.4f;
        d_accI += I;
        d_accQ += Q;
        if (++d_cntIF < dif) continue;
        float fi = d_accI * inv_if, fq = d_accQ * inv_if;
        d_accI = d_accQ = 0;
        d_cntIF = 0;

        ifpow += fi * fi + fq * fq;
        nif++;

        float ang;
        if (am) {
            /* Envelope of the complex baseband. AM carries its information in
             * the magnitude, so there is no discriminator and no de-emphasis:
             * the carrier plus modulation IS the demodulated signal, offset by
             * the carrier's own amplitude, which the DC tracker removes. */
            float env = sqrtf(fi * fi + fq * fq);
            d_dc += AM_DC_ALPHA * (env - d_dc);
            ang = env - d_dc;
        } else {
            float dre = fi * d_pI + fq * d_pQ;
            float dim = fq * d_pI - fi * d_pQ;
            ang = atan2f(dim, dre);
        }
        d_pI = fi;
        d_pQ = fq;

        d_accAud += ang;
        if (++d_cntAud < daud) continue;
        float a = d_accAud * inv_aud;
        d_accAud = 0;
        d_cntAud = 0;

        float out;
        if (dea > 0) { d_deemph += dea * (a - d_deemph); out = d_deemph; }
        else out = a;
        int v = open ? (int)(out * gain) : 0;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        aud[naud++] = (int16_t)v;
    }
    if (naud) audio_play(aud, naud);

    if (nif) {
        float lvl = 10.0f * log10f(ifpow / nif + 1e-6f);
        s_sig_db += 0.1f * (lvl - s_sig_db); /* smooth */
        /* noise floor: instant down, ~1 min to climb fully */
        if (!s_nf_seeded) { s_nf_seeded = true; s_nf_db = lvl; }
        else if (lvl < s_nf_db) s_nf_db = lvl;
        else s_nf_db += 3e-4f * (lvl - s_nf_db);
    }
}

static void fft_block(void)
{
    for (int i = 0; i < FFT_N; i++) {
        float wi = s_window[i];
        s_fft[2 * i] = ((float)s_blk[2 * i] - 127.4f) * wi;
        s_fft[2 * i + 1] = ((float)s_blk[2 * i + 1] - 127.4f) * wi;
    }
    dsps_fft2r_fc32(s_fft, FFT_N);
    dsps_bit_rev_fc32(s_fft, FFT_N);
    for (int k = 0; k < FFT_N; k++) {
        float re = s_fft[2 * k], im = s_fft[2 * k + 1];
        s_acc[k] += re * re + im * im;
    }
    s_acc_cnt++;
}

static void snapshot(void)
{
    int w = (s_ready + 1) & 1;
    float *o = s_spec[w];
    for (int i = 0; i < FFT_N; i++) {
        int bin = (i + FFT_N / 2) & (FFT_N - 1); /* fftshift: DC to centre */
        o[i] = 10.0f * log10f(s_acc[bin] / (float)s_acc_cnt + 1e-6f)
             + SPECTRUM_DBM_OFFSET;
    }
    s_ready = w;
    memset(s_acc, 0, sizeof(s_acc));
    s_acc_cnt = 0;
}

static void iq_proc_task(void *arg)
{
    (void)arg;
    size_t have = 0;
    int blocks = 0;
    while (1) {
        if (s_flush_req) { /* ui retuned: drop stale accumulation */
            s_flush_req = false;
            memset(s_acc, 0, sizeof(s_acc));
            s_acc_cnt = 0;
            s_ready = -1;
        }
        size_t n = sdr_read(s_blk + have, sizeof(s_blk) - have);
        have += n;
        if (have < sizeof(s_blk)) {
            if (n == 0) vTaskDelay(1);
            continue;
        }
        have = 0;
        demod_block();
        if (++blocks >= FFT_EVERY) {
            blocks = 0;
            fft_block();
            if (s_acc_cnt >= FFT_PER_FRAME) snapshot();
        }
    }
}

void dsp_start(void)
{
    dsps_fft2r_init_fc32(NULL, FFT_N);
    dsps_wind_hann_f32(s_window, FFT_N);
    memset(s_acc, 0, sizeof(s_acc));
    ESP_LOGI(TAG, "FM demod + FFT %d starting", FFT_N);
    xTaskCreatePinnedToCore(iq_proc_task, "iq_proc", 16384, NULL, 12, NULL, 1);
}

int dsp_spectrum_bins(void) { return FFT_N; }

const float *dsp_spectrum(void)
{
    int r = s_ready;
    return (r < 0) ? NULL : s_spec[r];
}

void dsp_set_squelch_db(float db) { s_squelch_db = db; }
float dsp_get_squelch_db(void) { return s_squelch_db; }

void dsp_set_mute(bool mute) { s_mute = mute; }
bool dsp_get_mute(void) { return s_mute; }
float dsp_signal_db(void) { return s_sig_db; }
float dsp_noise_floor_db(void) { return s_nf_db; }

void dsp_flush_spectrum(void) { s_flush_req = true; }

void dsp_set_mode(int mode)
{
    if (mode < DSP_MODE_WFM || mode >= DSP_MODE_COUNT) mode = DSP_MODE_WFM;
    if (mode == s_mode) return;

    switch (mode) {
    case DSP_MODE_NFM:
        s_decim_if = 24; s_decim_audio = 1;       /* 1.152M/24 = 48 kHz IF */
        s_deemph_a = 0.0f; s_gain = 18000.0f;
        break;
    case DSP_MODE_AM:
        /* Same 48 kHz IF as NFM, but the demodulator output is an amplitude in
         * ADC counts rather than an angle in radians, so the gain has to be far
         * lower — reusing NFM's 18000 would clip on the first syllable. */
        s_decim_if = 24; s_decim_audio = 1;
        s_deemph_a = 0.0f; s_gain = 4000.0f;
        break;
    default:
        s_decim_if = 4; s_decim_audio = 6;        /* 288 kHz IF → 48 kHz audio */
        s_deemph_a = 0.24f; s_gain = 9000.0f;
        break;
    }
    s_inv_if = 1.0f / s_decim_if;
    s_inv_audio = 1.0f / s_decim_audio;
    /* reset decimator/discriminator state to avoid a transient */
    d_accI = d_accQ = d_accAud = 0;
    d_cntIF = d_cntAud = 0;
    d_pI = d_pQ = d_deemph = 0;
    d_dc = 0;
    s_mode = mode;
}
int dsp_get_mode(void) { return s_mode; }
