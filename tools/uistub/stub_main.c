/*
 * stub_main.c — host-side preview harness for main/ui.c.
 *
 * Purpose: iterate on layout without a 30-second flash cycle. It links the REAL
 * ui.c against stub implementations of everything it calls, renders a few
 * hundred frames into a fake panel framebuffer, and writes a PPM.
 *
 * The stub PPA performs the actual scale + rotate (and honours 270° for the
 * IMU flip), so the output is what the glass shows — not the raw landscape
 * buffer. That matters: a bug in the rotation or in the touch mapping would
 * otherwise be invisible here.
 *
 * See tools/uistub/README.md for usage.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_tab5.h"
#include "sdr.h"
#include "dsp.h"
#include "audio.h"
#include "ui.h"
#include "driver/ppa.h"

#define LW 640
#define LH 360
#define PANEL_W 720
#define PANEL_H 1280
#define SCALE 2

/* ------------------------------------------------------------------ display */

static uint16_t g_panel[PANEL_W * PANEL_H];
static uint16_t g_last[PANEL_W * PANEL_H]; /* last frame actually flushed */
static int g_presents;

void *bsp_tab5_display_get_frame_buffer(int fb_index)
{
    (void)fb_index;
    return g_panel;
}

void bsp_tab5_display_flush(int fb_index)
{
    (void)fb_index;
    memcpy(g_last, g_panel, sizeof g_last);
    g_presents++;
}

/*
 * Fake PPA: undo exactly the mapping ui.c uses for touch, which is the inverse
 * of what the hardware does. Deriving it from the same equations guarantees the
 * preview and the touch tests agree with each other.
 *
 *   90°  (upright): lx = (PANEL_H - py)/S, ly = px/S
 *   270° (flipped): lx = py/S,           ly = (PANEL_W - px)/S
 */
void ppa_do_scale_rotate_mirror(ppa_client_handle_t h, ppa_srm_oper_config_t *c)
{
    (void)h;
    const uint16_t *src = c->in.buffer;
    uint16_t *dst = c->out.buffer;
    int s = (int)c->scale_x;
    if (s < 1) s = 1;

    bool flip = (c->rotation_angle == PPA_SRM_ROTATION_ANGLE_270);

    for (int py = 0; py < PANEL_H; py++) {
        for (int px = 0; px < PANEL_W; px++) {
            int lx, ly;
            if (flip) {
                lx = py / s;
                ly = (PANEL_W - px) / s;
            } else {
                lx = (PANEL_H - py) / s;
                ly = px / s;
            }
            if (lx < 0) lx = 0;
            if (lx >= LW) lx = LW - 1;
            if (ly < 0) ly = 0;
            if (ly >= LH) ly = LH - 1;
            dst[(size_t)py * PANEL_W + px] = src[(size_t)ly * LW + lx];
        }
    }
}

/* ------------------------------------------------------------ time and tasks */

static int64_t g_us;
static void (*g_task)(void *);

int64_t esp_timer_get_time(void) { return g_us; }

BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack,
                                   void *arg, unsigned prio, void *handle, int core)
{
    (void)name; (void)stack; (void)prio; (void)handle; (void)core;
    g_task = fn;
    (void)arg;
    return pdPASS;
}

/* --------------------------------------------------------------- touch input */

/* Scripted taps: "at,lx,ly[,hold]" separated by ';'. `at` is the poll index at
 * which the finger lands, so two taps in one run can drive the keypad (open it,
 * then press a key). */
typedef struct {
    int at, lx, ly, hold;
} tap_t;

#define MAX_TAPS 8
static tap_t g_taps[MAX_TAPS];
static int g_ntaps;
static int g_touch_polls;

/* The firmware needs 2 consecutive pressed polls to register an edge, so a tap
 * must dwell. Holding for >= 3 polls is enough; the release between taps is
 * what creates the next edge. */
#define TAP_MIN_HOLD 3

int bsp_tab5_touch_read(esp_lcd_touch_point_data_t *points, uint8_t max_points)
{
    (void)max_points;
    int i = g_touch_polls++;

    for (int k = 0; k < g_ntaps; k++) {
        tap_t *t = &g_taps[k];
        int hold = t->hold < TAP_MIN_HOLD ? TAP_MIN_HOLD : t->hold;
        if (i < t->at || i >= t->at + hold) continue;

        /* Inverse of the touch mapping ui.c applies (upright orientation). */
        points[0].x = t->ly * SCALE;
        points[0].y = PANEL_H - t->lx * SCALE;
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------- device stubs */

static uint32_t g_center = 100000000u;
static int g_mode;
static bool g_imu_flipped;
static int g_vol = 8;

void sdr_get_stats(sdr_stats_t *o)
{
    memset(o, 0, sizeof *o);
    o->sample_rate = 1152000;
    o->center_freq = g_center;
    o->tuner = "Rafael Micro R820T";
    o->present = true;
    o->produced_bytes = 1u << 20;
    o->dropped_bytes = 0;
}

int sdr_set_center_freq(uint32_t f) { (void)f; return 0; }
bool sdr_is_present(void) { return true; }

void audio_set_volume(int v) { (void)v; }

void bsp_tab5_battery_read(bsp_tab5_battery_t *o)
{
    memset(o, 0, sizeof *o);
    o->present = true;
    o->mv = 8120;
    o->ma = 0.0f;
    o->percent = 89;
    o->charging = false;
    o->external = true;
}

bool bsp_tab5_orientation_poll(void)
{
    /* Only report a change once, so the flip is exercised exactly like the IMU
     * would: ui.c calls present() itself when this returns true. */
    static bool reported;
    if (g_imu_flipped && !reported) { reported = true; return true; }
    return false;
}

void bsp_tab5_orientation_get(bsp_tab5_orient_t *o)
{
    memset(o, 0, sizeof *o);
    o->present = true;
    o->flipped = g_imu_flipped;
    o->az = -988;
    o->rz = -4046;
    o->range = 1;
}

/* ------------------------------------------------------------- DSP spectrum */

#define BINS 1024
static float g_spec[4][BINS];
static int g_frame;

int dsp_spectrum_bins(void) { return BINS; }
const float *dsp_spectrum(void) { return g_spec[g_frame & 3]; }
float dsp_signal_db(void) { return 30.0f; }
float dsp_noise_floor_db(void) { return 62.0f; }
void dsp_flush_spectrum(void) {}
void dsp_set_mode(int m) { g_mode = m; }
int dsp_get_mode(void) { return g_mode; }
void dsp_set_squelch_db(float d) { (void)d; }
float dsp_get_squelch_db(void) { return -200.0f; }

/*
 * Synthetic spectrum: a noise floor plus a couple of carriers.
 *
 * Values are produced on the SAME nominal-dBm scale the firmware's dsp.c
 * publishes (raw FFT dB plus its offset), because ui.c plots them directly and
 * no longer applies an offset of its own. Getting this wrong would make the
 * preview look plausible while disagreeing with the device.
 */
#define SPECTRUM_DBM_OFFSET (-168.0f)

static void synth(void)
{
    for (int fr = 0; fr < 4; fr++) {
        for (int i = 0; i < BINS; i++) {
            float x = (float)i / BINS;
            float n = 62.0f + 2.5f * sinf(x * 91.0f + fr) + 1.5f * sinf(x * 233.0f);
            float d = (x - 0.5f) * 2.0f;
            float st = 16.0f * expf(-(d * d) / (2 * 0.055f * 0.055f));
            st += 6.0f * expf(-(d * d) / (2 * 0.16f * 0.16f));
            st += 9.0f * expf(-powf(x - 0.28f, 2) / (2 * 0.004f * 0.004f));
            st += 7.0f * expf(-powf(x - 0.72f, 2) / (2 * 0.003f * 0.003f));
            st *= 1.0f + 0.06f * sinf(fr * 1.7f);
            g_spec[fr][i] = n + st + SPECTRUM_DBM_OFFSET;
        }
    }
}

/* -------------------------------------------------------------------- output */

static const char *g_out = "preview.ppm";
static int g_frames = 300;

static void write_ppm(void)
{
    FILE *f = fopen(g_out, "wb");
    if (!f) { perror(g_out); exit(1); }
    fprintf(f, "P6\n%d %d\n255\n", PANEL_W, PANEL_H);
    for (size_t i = 0; i < (size_t)PANEL_W * PANEL_H; i++) {
        uint16_t v = g_last[i];
        fputc((((v >> 11) & 0x1F) * 255) / 31, f);
        fputc((((v >> 5) & 0x3F) * 255) / 63, f);
        fputc(((v & 0x1F) * 255) / 31, f);
    }
    fclose(f);
}

void vTaskDelay(int t)
{
    (void)t;
    g_us += 8000;
    g_frame++; /* dsp.c would publish a new spectrum frame here */

    if (g_frame >= g_frames) {
        write_ppm();
        fprintf(stderr, "frames=%d presents=%d freq=%u mode=%d\n",
                g_frame, g_presents, (unsigned)ui_get_freq(), dsp_get_mode());
        exit(0);
    }
}

static void parse_taps(const char *s)
{
    while (*s && g_ntaps < MAX_TAPS) {
        int at, lx, ly, hold = 4;
        int n = sscanf(s, "%d,%d,%d,%d", &at, &lx, &ly, &hold);
        if (n < 3) break;
        g_taps[g_ntaps++] = (tap_t){ at, lx, ly, hold };
        const char *semi = strchr(s, ';');
        if (!semi) break;
        s = semi + 1;
    }
}

int main(void)
{
    const char *e;

    if ((e = getenv("FREQ")))    g_center = (uint32_t)strtoul(e, NULL, 10);
    if ((e = getenv("MODE")))    g_mode = atoi(e);
    if ((e = getenv("VOL")))     g_vol = atoi(e);
    if ((e = getenv("FRAMES")))  g_frames = atoi(e);
    if ((e = getenv("OUT")))     g_out = e;
    if ((e = getenv("FLIP")))    g_imu_flipped = atoi(e) != 0;
    if ((e = getenv("TAPS")))    parse_taps(e);

    /* Backwards-compatible single tap: TAP_LX/TAP_LY/TOUCH_AT. */
    if (!g_ntaps && (e = getenv("TAP_LX"))) {
        int lx = atoi(e);
        int ly = (getenv("TAP_LY") ? atoi(getenv("TAP_LY")) : 30);
        int at = (getenv("TOUCH_AT") ? atoi(getenv("TOUCH_AT")) : 20);
        int hold = (getenv("TOUCH_HOLD") ? atoi(getenv("TOUCH_HOLD")) : 4);
        g_taps[g_ntaps++] = (tap_t){ at, lx, ly, hold };
    }

    synth();
    ui_start();
    if (g_vol != 8) ui_set_volume(g_vol * 100 / 8);
    if (g_mode) ui_set_mode(g_mode);
    if (g_center != 100000000u) ui_set_freq(g_center);
    g_task(NULL); /* runs the render loop until vTaskDelay() ends the run */
    return 0;
}
