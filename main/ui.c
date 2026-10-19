/*
 * ui.c — instrument-style spectrum console for Tab5-SDR.
 *
 * Display-only build (no touch): the screen mirrors a classic SDR analyzer
 * head — a dense status header with a large 7-segment frequency readout, a
 * green filled spectrum with a 10 dB graticule labelled in dBm on both edges,
 * a frequency ruler, and a rainbow waterfall with a time axis.
 *
 * Rendering goes into a 640x360 RGB565 buffer in PSRAM; the P4 PPA scales it
 * x2 and rotates 90° onto the 720x1280 panel (half-res keeps the PPA cheap).
 *
 * Levels are uncalibrated: dsp.c reports dB relative to its own FFT scale, so
 * the window is anchored on each frame's noise floor and the dBm labels are
 * that window shifted to read like a calibrated analyzer.
 * Tuning input is not wired up yet — ui_set_freq()/ui_set_mode() are the hooks
 * a future encoder, keypad or serial console would call.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/ppa.h"

#include "bsp_tab5.h"
#include "sdr.h"
#include "dsp.h"
#include "audio.h"

static const char *TAG = "ui";

#define LW 640
#define LH 360
#define PANEL_W 720
#define PANEL_H 1280
#define SCALE 2

/* ---- layout (single column, top to bottom) ------------------------------- */
#define HDR_H    62    /* status header          0..61                       */
#define SPEC_Y0  64    /* spectrum plot         64..195                      */
#define SPEC_H   132
#define RULER_Y  198   /* ruler ticks + labels 198..210                      */
#define WF_Y0    214   /* waterfall            214..341                      */
#define WF_H     128
#define FOOT_Y   343   /* status footer        343..359                      */
#define GRID_DIV 8     /* 8 x 10 dB rows across the plot                     */

/* Touch target for the WFM/NFM badge. Bigger than the glyphs so it is
 * comfortably tappable on the physical panel (these are logical pixels; the
 * PPA scales them 2x on the way to the glass). */
#define MODE_BOX_X0 584
#define MODE_BOX_Y0  14
#define MODE_BOX_X1 638
#define MODE_BOX_Y1  50

/* Holding the mode badge for this many polls (≈8 ms each) declares the current
 * orientation upright. This is the escape hatch for a bad automatic latch: the
 * reference is normally learned and stored, but if it ever comes out inverted the
 * screen would otherwise stay wrong with no way back. */
#define MODE_LONG_POLLS 80

/* Scan transport, in the dead space between the frequency readout and the mode
 * badge (the readout ends near x=314, the badge starts at 584).
 *
 * Three buttons: play/stop, step back, step forward. They are deliberately not
 * a full transport bar — a spectrum analyzer has nothing to "rewind", so
 * back/forward mean "hunt for the next signal in that direction". */
#define SCAN_BTN_Y0   15
#define SCAN_BTN_Y1   49
#define SCAN_BTN_H    (SCAN_BTN_Y1 - SCAN_BTN_Y0)
#define SCAN_BTN_W    72
#define SCAN_BTN_GAP  12
#define SCAN_BTN_X0   328
#define SCAN_BTN_N    3

/* Scan behaviour. Steps are small relative to the 1.15 MHz view so a signal is
 * not skipped between steps; the tick rate gives the tuner and the demodulator
 * time to settle before the level is judged. */
#define SCAN_STEP_HZ   100000 /* 100 kHz per step                      */
#define SCAN_TICK_US   100000 /* 10 steps/s                            */
#define SCAN_HOLD_DB     6.0f /* this far above the noise floor = found */
#define SEEK_MAX_STEPS   400  /* give up after ~40 MHz                 */

/* Volume: a vertical 8-step bar in the right-hand gutter beside the waterfall.
 * That strip was dead space (the waterfall only paints PLOT_W columns, and the
 * dBm labels stop at the spectrum's bottom edge), so the control costs no
 * layout at all. Discrete steps rather than a continuous slider: coarse,
 * glanceable, and easy to hit with a finger.
 *
 * VOL_ZONE_X0 equals PLOT_X1 (34 + 574 = 608). */
#define VOL_SEGS    8
#define VOL_ZONE_X0 608            /* whole gutter is the touch target */
#define VOL_ZONE_X1 LW
#define VOL_BAR_X0  612
#define VOL_BAR_X1  634
#define VOL_BAR_TOP (WF_Y0 + 16)   /* leaves a row for the level digit */
#define VOL_BAR_BOT (WF_Y0 + WF_H - 2)

/* ---- level mapping ------------------------------------------------------- */
/* dsp_spectrum() already returns NOMINAL dBm — dsp.c applies its own offset when
 * a frame is published — so everything here works in that unit directly. DB_SPAN
 * is the only level knob that belongs to the UI: how many dB fit vertically.
 *
 * Do not mix this with dsp_signal_db(), which is demodulated IF power in raw dB
 * on a completely different scale.
 *
 * The plot normalises on the frame's own noise floor, so the trace stays on
 * screen whatever the dongle's gain is; the dBm labels are then just that window
 * shifted to look like a calibrated analyzer. */
#define DB_SPAN          80.0f   /* dB across the spectrum graticule        */
/* The waterfall uses a much narrower range than the spectrum graticule: a
 * 80 dB waterfall is almost black because everything real sits in the top
 * 20 dB. WF_SPAN + WF_BASE gives noise a visible blue base and 20 dB of signal
 * a bright green/white core, like a hardware analyzer. */
#define WF_SPAN          45.0f   /* dB from waterfall floor to ceiling      */
#define WF_BASE           0.16f  /* colour index of the noise floor         */
#define REF_SLEW          0.12f  /* ref-level follow rate (per frame)       */
#define PEAK_DECAY        0.35f  /* peak-hold fall, dB per frame            */

#define FREQ_MIN 24000000u
#define FREQ_MAX 1766000000u

/* ---- numeric keypad (tap the frequency readout to open) ------------------ */
/* Values are entered in MHz with a decimal point ("121.5"), which is how
 * frequencies are actually quoted; typing nine raw Hz digits would be slow and
 * easy to get wrong by a factor of ten. The panel is modal: while it is open it
 * swallows every touch and is painted last, so it sits above the live spectrum. */
#define KP_X0      148
#define KP_Y0       14
#define KP_W       344
#define KP_H       332
#define KP_PAD       8
#define KP_KEY_W   104
#define KP_KEY_H    48
#define KP_GAP       8
#define KP_VAL_Y   (KP_Y0 + KP_PAD)               /* value readout top */
#define KP_VAL_H    40
#define KP_GRID_Y  (KP_VAL_Y + KP_VAL_H + KP_GAP) /* first key row     */
#define KP_BTN_Y   (KP_GRID_Y + 4 * (KP_KEY_H + KP_GAP))
#define KP_BTN_H    44
#define KP_BTN_W   160

/* Tap target over the 7-segment readout (it occupies x 8..~320, y 15..48).
 * Kept clear of the mode badge on the right so the two never fight. */
#define FREQ_BOX_X0   4
#define FREQ_BOX_X1 322   /* the readout ends near x=314; keep clear of the scan buttons at 328 */
#define FREQ_BOX_Y0   8
#define FREQ_BOX_Y1  52

/* ---- theme --------------------------------------------------------------- */
#define RGB565(r, g, b) \
    ((uint16_t)(((((r) >> 3) & 0x1F) << 11) | ((((g) >> 2) & 0x3F) << 5) | (((b) >> 3) & 0x1F)))
#define COL_BG      RGB565(0, 0, 0)
#define COL_GRID    RGB565(0, 64, 0)
#define COL_GRIDV   RGB565(0, 44, 0)
#define COL_FILL    RGB565(0, 74, 18)    /* body under the spectrum curve   */
#define COL_TRACE   RGB565(0, 255, 64)   /* bright curve                    */
#define COL_PEAK    RGB565(0, 120, 84)   /* peak hold                       */
#define COL_TEXT    RGB565(150, 178, 150)
#define COL_DIM     RGB565(96, 118, 96)
#define COL_GREEN   RGB565(0, 235, 96)
#define COL_YELLOW  RGB565(240, 215, 70)
#define COL_CYAN    RGB565(90, 215, 235)
#define COL_MAGENTA RGB565(235, 90, 215)
#define COL_RED     RGB565(240, 70, 70)
#define COL_WHITE   RGB565(238, 242, 238)
#define COL_CURSOR  RGB565(255, 190, 60)
#define COL_SEG_OFF RGB565(26, 32, 27) /* unlit 7-seg segment (VFD ghost)  */

static uint16_t *LBUF;
static ppa_client_handle_t s_ppa;
static int s_fb_index;
static uint16_t s_wlut[256]; /* waterfall LUT: black -> magenta -> cyan -> white */

static uint32_t s_center;
static int s_vol_level = 8; /* 1..VOL_SEGS (8 discrete steps); boot at max */
static int s_mode;   /* DSP_MODE_* currently selected                 */
static int s_fps;
static bsp_tab5_battery_t s_bat; /* refreshed every ~2 s from the INA226 */
static float s_ref_db = -30.0f; /* top of the dB window (nominal dBm) */
static int s_ref_shown = -1000; /* ref value currently painted in the header */
static bool s_ref_init;
static float s_nf_dbm = -110.0f; /* frame noise floor (nominal dBm)     */
static float s_peak_dbm;         /* frame peak, for the header readout  */
static float s_peak[LW];  /* peak-hold level per column (dBm)       */
static volatile bool s_redraw; /* set by ui_set_freq()/ui_set_mode()  */

/* Touch state. The panel reports portrait coordinates, which are mapped back
 * into this landscape buffer by the ui_task loop. */
static bool s_mode_hot;   /* finger is currently on the WFM/NFM badge     */
static int  s_mode_hold;  /* polls the badge has been held               */
static int  s_scan_hot;   /* scan button under the finger, -1 = none     */

/* Scan transport state. Both arrows share one model:
 *   s_scan_dir  0 = idle, -1 = hunting down, +1 = hunting up
 *   s_scan_left steps remaining before the hunt gives up
 * The middle button is NOT part of this: it latches the demodulator mute and
 * has nothing to do with where the receiver is tuned. A manual tune cancels a
 * hunt, so the user can always take back control. */
static int  s_scan_dir;
/* Set when the receiver has just been stopped; consumed by ui_task AFTER it has
 * drawn the current frame, so the blank-out cannot be overwritten by the
 * spectrum pointer fetched earlier in the same iteration. */
static bool s_rx_off_paint;
static int  s_scan_left;
static int64_t s_scan_next;
static bool s_vol_hot;    /* finger is currently on the volume bar       */

/* Screen orientation. Driven by the IMU: when the device is turned upside down
 * the PPA rotation switches from 90° to 270° and the touch mapping mirrors to
 * match (see present() and handle_touch()). */
static bool s_flipped;

/* Numeric keypad state. */
static bool s_kp_open;
static char s_kp_buf[24];  /* digits typed, e.g. "121.5"                  */
static int  s_kp_len;
static int  s_kp_hot = -1; /* key index under the finger, for feedback     */
static bool s_kp_closed;   /* overlay just closed: repaint underneath      */

static const char *const KP_LABELS[12] = {
    "1", "2", "3",
    "4", "5", "6",
    "7", "8", "9",
    ".", "0", "DEL",
};

/* ---- primitives ---------------------------------------------------------- */

static void rect(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LW) w = LW - x;
    if (y + h > LH) h = LH - y;
    if (w <= 0 || h <= 0) return;
    for (int j = 0; j < h; j++) {
        uint16_t *row = LBUF + (size_t)(y + j) * LW + x;
        for (int i = 0; i < w; i++) row[i] = c;
    }
}

static void hline(int x, int y, int w, uint16_t c) { rect(x, y, w, 1, c); }
static void vline(int x, int y, int h, uint16_t c) { rect(x, y, 1, h, c); }

/* ---- 5x7 bitmap font, full printable ASCII ------------------------------- */

static const uint8_t FONT[][7] = {
    /* ' ' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* '!' */ {0x04,0x04,0x04,0x04,0x04,0x00,0x04},
    /* '"' */ {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00},
    /* '#' */ {0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00},
    /* '$' */ {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
    /* '%' */ {0x11,0x12,0x04,0x08,0x15,0x12,0x11},
    /* '&' */ {0x0C,0x12,0x14,0x08,0x15,0x12,0x0D},
    /* ''' */ {0x04,0x04,0x00,0x00,0x00,0x00,0x00},
    /* '(' */ {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    /* ')' */ {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    /* '*' */ {0x04,0x15,0x0E,0x1F,0x0E,0x15,0x04},
    /* '+' */ {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    /* ',' */ {0x00,0x00,0x00,0x00,0x0C,0x04,0x08},
    /* '-' */ {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    /* '.' */ {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C},
    /* '/' */ {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    /* '0' */ {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* '1' */ {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    /* '2' */ {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    /* '3' */ {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    /* '4' */ {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    /* '5' */ {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    /* '6' */ {0x0E,0x11,0x10,0x1E,0x11,0x11,0x0E},
    /* '7' */ {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    /* '8' */ {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* '9' */ {0x0E,0x11,0x11,0x0F,0x01,0x11,0x0E},
    /* ':' */ {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00},
    /* ';' */ {0x00,0x04,0x04,0x00,0x04,0x04,0x08},
    /* '<' */ {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    /* '=' */ {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    /* '>' */ {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
    /* '?' */ {0x0E,0x11,0x01,0x06,0x04,0x00,0x04},
    /* '@' */ {0x0E,0x11,0x15,0x15,0x17,0x10,0x0E},
    /* 'A' */ {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 'B' */ {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    /* 'C' */ {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    /* 'D' */ {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C},
    /* 'E' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    /* 'F' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    /* 'G' */ {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
    /* 'H' */ {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 'I' */ {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* 'J' */ {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    /* 'K' */ {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    /* 'L' */ {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    /* 'M' */ {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    /* 'N' */ {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    /* 'O' */ {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 'P' */ {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    /* 'Q' */ {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    /* 'R' */ {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    /* 'S' */ {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E},
    /* 'T' */ {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    /* 'U' */ {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 'V' */ {0x11,0x11,0x11,0x11,0x11,0x0A,0x04},
    /* 'W' */ {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    /* 'X' */ {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    /* 'Y' */ {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    /* 'Z' */ {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    /* '[' */ {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
    /* '\' */ {0x10,0x10,0x08,0x04,0x02,0x01,0x01},
    /* ']' */ {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
    /* '^' */ {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
    /* '_' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    /* '`' */ {0x08,0x04,0x02,0x00,0x00,0x00,0x00},
    /* 'a' */ {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F},
    /* 'b' */ {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E},
    /* 'c' */ {0x00,0x00,0x0E,0x11,0x10,0x11,0x0E},
    /* 'd' */ {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F},
    /* 'e' */ {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E},
    /* 'f' */ {0x06,0x09,0x08,0x1C,0x08,0x08,0x08},
    /* 'g' */ {0x00,0x00,0x0F,0x11,0x11,0x0F,0x1E},
    /* 'h' */ {0x10,0x10,0x1E,0x12,0x12,0x12,0x12},
    /* 'i' */ {0x04,0x00,0x04,0x04,0x04,0x04,0x04},
    /* 'j' */ {0x04,0x00,0x04,0x04,0x04,0x14,0x0C},
    /* 'k' */ {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
    /* 'l' */ {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* 'm' */ {0x00,0x00,0x1E,0x15,0x15,0x11,0x11},
    /* 'n' */ {0x00,0x00,0x1E,0x12,0x12,0x12,0x12},
    /* 'o' */ {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E},
    /* 'p' */ {0x1F,0x11,0x11,0x1F,0x10,0x10,0x10},
    /* 'q' */ {0x0F,0x11,0x11,0x0F,0x01,0x01,0x01},
    /* 'r' */ {0x00,0x00,0x16,0x19,0x10,0x10,0x10},
    /* 's' */ {0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E},
    /* 't' */ {0x08,0x08,0x1C,0x08,0x08,0x09,0x0E},
    /* 'u' */ {0x00,0x00,0x12,0x12,0x12,0x13,0x0D},
    /* 'v' */ {0x00,0x00,0x11,0x11,0x11,0x0A,0x04},
    /* 'w' */ {0x00,0x00,0x11,0x15,0x15,0x1B,0x0A},
    /* 'x' */ {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11},
    /* 'y' */ {0x00,0x00,0x11,0x11,0x0F,0x01,0x1E},
    /* 'z' */ {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F},
    /* '{' */ {0x06,0x04,0x04,0x08,0x04,0x04,0x06},
    /* '|' */ {0x04,0x04,0x04,0x00,0x04,0x04,0x04},
    /* '}' */ {0x0C,0x08,0x08,0x04,0x08,0x08,0x0C},
    /* '~' (maps to ±) */ {0x00,0x04,0x0E,0x04,0x00,0x1F,0x00},
};

static const uint8_t *glyph(char c)
{
    if (c < ' ' || c > '~') return FONT[0];
    return FONT[c - ' '];
}

static void text(int x, int y, int sc, uint16_t c, const char *s)
{
    for (; *s; s++) {
        const uint8_t *g = glyph(*s);
        for (int row = 0; row < 7; row++)
            for (int col = 0; col < 5; col++)
                if (g[row] & (1 << (4 - col))) rect(x + col * sc, y + row * sc, sc, sc, c);
        x += 6 * sc;
    }
}
static int text_w(const char *s, int sc) { return (int)strlen(s) * 6 * sc; }

/* Right-aligned text: `x` is the right edge. */
static void text_r(int x, int y, int sc, uint16_t c, const char *s)
{
    text(x - text_w(s, sc), y, sc, c, s);
}

/* Centred text: `x` is the centre. */
static void text_c(int x, int y, int sc, uint16_t c, const char *s)
{
    text(x - text_w(s, sc) / 2, y, sc, c, s);
}

/* Seven-segment digit, drawn as `t`-thick strokes. */
/* ---- 7-segment readout --------------------------------------------------- */

/* Filled convex polygon (scanline). The 45-degree chamfered ends this produces
 * are what make the readout look like a real VFD/LCD instead of stacked
 * rectangles. */
static void poly(const int *px, const int *py, int n, uint16_t c)
{
    int ymin = py[0], ymax = py[0];
    for (int i = 1; i < n; i++) {
        if (py[i] < ymin) ymin = py[i];
        if (py[i] > ymax) ymax = py[i];
    }
    for (int y = ymin; y <= ymax; y++) {
        float yc = (float)y + 0.5f, lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            float y0 = (float)py[i], y1 = (float)py[j];
            if (y0 == y1) continue;
            if (yc < fminf(y0, y1) || yc >= fmaxf(y0, y1)) continue;
            float x = (float)px[i] + (yc - y0) * (float)(px[j] - px[i]) / (y1 - y0);
            if (x < lo) lo = x;
            if (x > hi) hi = x;
        }
        if (hi <= lo) continue;
        int xa = (int)lroundf(lo), xb = (int)lroundf(hi);
        if (xb > xa) rect(xa, y, xb - xa, 1, c);
    }
}

/* Horizontal bar spanning [x0,x1], both ends chamfered by t/2. */
static void bar_h(int x0, int x1, int y, int t, uint16_t c)
{
    int h = t / 2;
    const int xs[6] = { x0 + h, x1 - h, x1, x1 - h, x0 + h, x0 };
    const int ys[6] = { y,      y,      y + h, y + t, y + t, y + h };
    poly(xs, ys, 6, c);
}

/* Vertical bar spanning [y0,y1], both ends chamfered by t/2. */
static void bar_v(int x, int y0, int y1, int t, uint16_t c)
{
    int h = t / 2;
    const int xs[6] = { x,      x + h, x + t, x + t, x + h, x };
    const int ys[6] = { y0 + h, y0,    y0 + h, y1 - h, y1,   y1 - h };
    poly(xs, ys, 6, c);
}

/* One digit. The unlit segments are drawn first at very low intensity, the way
 * a real VFD looks, then the lit ones are drawn over them. Segments are inset
 * by a 1 px gap so neighbouring bars read as separate strokes. */
static void seg7_digit(int x, int y, int dw, int dh, int t, int d, uint16_t c)
{
    static const uint8_t M[10] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
    const int h  = t / 2;      /* chamfer depth at each stroke end */
    const int g  = 1;          /* gap between adjacent segments    */
    const int ym = y + dh / 2; /* vertical centre                  */

    /* bar_h()/bar_v() chamfer their own ends inward by h, so the OUTER extent
     * must have h added back. The full-thickness part then lands exactly on the
     * intended span (x+t+g .. x+dw-t-g) instead of stopping h short, which is
     * what previously squeezed every bar into a diamond and left the middle of
     * the digit hollow. */
    const int hx0 = x + t - h + g;              /* A/D/G left  extent */
    const int hx1 = x + dw - t + h - g;         /* A/D/G right extent */
    const int gtop = ym - h;                    /* G bar top edge     */
    const int gbot = gtop + t;                  /* G bar bottom edge  */
    const int vy0 = y + t - h + g;              /* upper verticals: top    */
    const int vy1 = gtop - g + h;               /*                  bottom */
    const int vy2 = gbot + g - h;               /* lower verticals: top    */
    const int vy3 = y + dh - t + h - g;         /*                  bottom */
    const int xr = x + dw - t;                  /* right-hand verticals    */
    const int m = M[d];

    for (int pass = 0; pass < 2; pass++) {
        uint16_t col = pass ? c : COL_SEG_OFF;
        int on = pass ? m : 0x7F;
        if (on & 1)  bar_h(hx0, hx1, y, t, col);           /* A */
        if (on & 2)  bar_v(xr, vy0, vy1, t, col);          /* B */
        if (on & 4)  bar_v(xr, vy2, vy3, t, col);          /* C */
        if (on & 8)  bar_h(hx0, hx1, y + dh - t, t, col);  /* D */
        if (on & 16) bar_v(x, vy2, vy3, t, col);           /* E */
        if (on & 32) bar_v(x, vy0, vy1, t, col);           /* F */
        if (on & 64) bar_h(hx0, hx1, gtop, t, col);        /* G */
    }
}

/* ---- waterfall colour map ------------------------------------------------ */

/* Classic SDR rainbow: black -> purple -> blue -> cyan -> green -> yellow ->
 * red -> white. Bright, high-contrast, and it reads as a waterfall at a glance. */
static void build_wlut(void)
{
    static const struct { float t; int r, g, b; } STOPS[] = {
        {0.00f,   0,   0,  16},
        {0.12f,  28,   0,  86},
        {0.28f,   0,  44, 176},
        {0.44f,   0, 168, 208},
        {0.58f,   0, 202,  92},
        {0.72f, 232, 222,  44},
        {0.86f, 242,  92,  28},
        {1.00f, 255, 255, 255},
    };
    const int n = (int)(sizeof(STOPS) / sizeof(STOPS[0]));
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;
        int k = 0;
        while (k < n - 2 && t > STOPS[k + 1].t) k++;
        float span = STOPS[k + 1].t - STOPS[k].t;
        float u = span > 0 ? (t - STOPS[k].t) / span : 0;
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        int r = STOPS[k].r + (int)((STOPS[k + 1].r - STOPS[k].r) * u);
        int g = STOPS[k].g + (int)((STOPS[k + 1].g - STOPS[k].g) * u);
        int b = STOPS[k].b + (int)((STOPS[k + 1].b - STOPS[k].b) * u);
        s_wlut[i] = RGB565(r, g, b);
    }
}

/* ---- level helpers ------------------------------------------------------- */

/* Map a spectrum bin (already nominal dBm) onto a screen row. */
static int level_to_y(float dbm)
{
    float t = (s_ref_db - dbm) / DB_SPAN; /* 0 at the top, 1 at the bottom */
    int y = SPEC_Y0 + (int)(t * SPEC_H);
    if (y < SPEC_Y0) y = SPEC_Y0;
    if (y > SPEC_Y0 + SPEC_H - 1) y = SPEC_Y0 + SPEC_H - 1;
    return y;
}

/* ---- battery ------------------------------------------------------------- */

/* Icon + percentage + pack voltage. Drawn in the header's first row, which is
 * otherwise just uptime/sample-rate/tuner, so it stays visible in every mode
 * including the no-dongle screen. */
static void draw_battery(void)
{
    const int x = 190, y = 2;

    if (!s_bat.present) {
        /* No INA226 answered: say so rather than showing a fake 0%. */
        text(x, y + 3, 1, COL_DIM, "NO BATT");
        return;
    }

    const int w = 24, h = 12;
    int pct = s_bat.percent;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    uint16_t col = s_bat.charging ? COL_CYAN
                 : (pct > 50 ? COL_GREEN : pct > 20 ? COL_YELLOW : COL_RED);

    /* 1px outline, terminal nub, then a proportional fill inset by 2px */
    rect(x, y, w, h, COL_DIM);
    rect(x + 1, y + 1, w - 2, h - 2, COL_BG);
    rect(x + w, y + h / 2 - 3, 2, 6, COL_DIM);
    int fill = ((w - 4) * pct + 50) / 100;
    if (fill > 0) rect(x + 2, y + 2, fill, h - 4, col);

    char b[16];
    snprintf(b, sizeof b, "%d%%", pct);
    text(x + 30, y + 3, 1, col, b);

    /* 2S pack, so ~6.4 V empty .. 8.4 V full */
    snprintf(b, sizeof b, "%.1fV", s_bat.mv / 1000.0f);
    text(x + 56, y + 3, 1, COL_DIM, b);

    if (s_bat.charging) text(x + 88, y + 3, 1, COL_CYAN, "CHG");
}

/* ---- header -------------------------------------------------------------- */

#define PLOT_X0  34
#define PLOT_W   574
#define PLOT_X1  (PLOT_X0 + PLOT_W)

/* ---- scan transport ------------------------------------------------------ */
/*
 * Three buttons in the dead space between the frequency readout (ends near
 * x=314) and the mode badge (starts at 584).
 *
 * They are not a media transport: a spectrum analyzer has nothing to rewind.
 * Instead, left to right:
 *   [0] seek down    hunt downward and park on the first signal found
 *   [1] play / stop  run/stop the receiver itself (does NOT retune)
 *   [2] seek up      same as [0], upward
 *
 * Seek is what the arrows are for: with 1.15 MHz on screen, nudging by one
 * step is already covered by dragging, so hunting is the useful behaviour.
 * The middle button is not a sweep: the arrows own frequency, this one owns
 * whether the receiver works at all. Stopping is a real stop — sdr releases the
 * dongle, so no IQ is read and the plot goes to the same static "no data"
 * state as an unplugged receiver. It is not a mute.
 */
static void scan_btn_x(int idx, int *x)
{
    *x = SCAN_BTN_X0 + idx * (SCAN_BTN_W + SCAN_BTN_GAP);
}

/* Button index at (lx,ly), or -1. */
static int scan_hit(int lx, int ly)
{
    if (ly < SCAN_BTN_Y0 || ly >= SCAN_BTN_Y1) return -1;
    for (int i = 0; i < SCAN_BTN_N; i++) {
        int x;
        scan_btn_x(i, &x);
        if (lx >= x && lx < x + SCAN_BTN_W) return i;
    }
    return -1;
}

static void draw_scan_buttons(void)
{
    /* Left-to-right: hunt down, play/stop decode, hunt up. The middle button
     * does not move the receiver at all — it latches the demodulator mute, so
     * the arrows are the only things that change frequency. */
    bool playing = sdr_get_running();

    for (int i = 0; i < SCAN_BTN_N; i++) {
        int x;
        scan_btn_x(i, &x);

        bool hot = (s_scan_hot == i);
        /* Light whichever button owns the current state, so the panel says
         * what is happening without a separate indicator. */
        bool active = (i == 0) ? (s_scan_dir < 0)
                    : (i == 1) ? playing
                               : (s_scan_dir > 0);

        uint16_t fg = hot ? COL_BG : (active ? COL_CURSOR : COL_DIM);
        rect(x, SCAN_BTN_Y0, SCAN_BTN_W, SCAN_BTN_H,
             hot ? COL_CURSOR : RGB565(0, 18, 10));

        int cx = x + SCAN_BTN_W / 2;
        int cy = (SCAN_BTN_Y0 + SCAN_BTN_Y1) / 2;

        if (i == 0) {
            int px[3] = { cx + 6, cx - 6, cx + 6 }; /* left */
            int py[3] = { cy - 8, cy,      cy + 8 };
            poly(px, py, 3, fg);
        } else if (i == 1) {
            /* Show the action the press will perform: a stop bar while audio is
             * running, a play triangle while it is muted. */
            if (playing) {
                rect(cx - 7, cy - 7, 14, 14, fg); /* stop */
            } else {
                int px[3] = { cx - 6, cx + 8, cx - 6 }; /* play */
                int py[3] = { cy - 8, cy,      cy + 8 };
                poly(px, py, 3, fg);
            }
        } else {
            int px[3] = { cx - 6, cx + 6, cx - 6 }; /* right */
            int py[3] = { cy - 8, cy,      cy + 8 };
            poly(px, py, 3, fg);
        }
    }
}

static void draw_header(void)
{
    sdr_stats_t st;
    sdr_get_stats(&st);

    rect(0, 0, LW, HDR_H, COL_BG);

    /* --- row 1: uptime, sample rate, tuner ------------------------------- */
    int64_t up = esp_timer_get_time() / 1000000;
    char b[48];
    snprintf(b, sizeof b, "UP %02d:%02d:%02d",
             (int)(up / 3600) % 100, (int)(up / 60) % 60, (int)(up % 60));
    text(8, 3, 1, COL_DIM, b);

    snprintf(b, sizeof b, "%.3f MSPS", st.sample_rate / 1e6);
    text(112, 3, 1, COL_DIM, b);

    /* Tuner name, or an explicit "NO DONGLE" — the dongle shares the USB port
     * with the flashing link, so its absence is normal and must be legible. */
    if (st.present) text_r(632, 3, 1, COL_DIM, st.tuner);
    else            text_r(632, 3, 1, COL_YELLOW, "NO DONGLE");

    draw_battery();

    /* --- row 2: big frequency readout, grouped MHz.kHz.Hz ----------------- */
    char d[16];
    snprintf(d, sizeof d, "%09u", (unsigned)s_center); /* 89500000 -> 089.500.000 */
    int nd = (int)strlen(d);
    /* Thick strokes and chamfered ends read as an instrument panel; thin square
     * bars just look like a calculator. The cell is sized to sit strictly
     * between row 1 (y<=14) and the parameter strip (y=50), so it cannot
     * collide with either. */
    const int dw = 23, dh = 33, t = 5, gap = 7, dotw = 12;
    int x = 8;  /* left-aligned: the readout anchors the header like a real
                 * analyzer, with the mode badge pushed out to the right. */
    int y = 15;
    for (int i = 0; i < nd; i++) {
        if (i && (nd - i) % 3 == 0) { /* thousands group separator */
            rect(x + 3, y + dh - 12, 5, 5, COL_DIM);
            x += dotw;
        }
        seg7_digit(x, y, dw, dh, t, d[i] - '0', COL_WHITE);
        x += dw + gap;
    }
    text(x, y + dh - 11, 1, COL_DIM, "Hz");
    /* --- mode badge (touch target: tap to toggle WFM/NFM) ---------------- */
    {
        uint16_t fg;
        const char *lab;
        switch (s_mode) {
        case DSP_MODE_AM:  fg = COL_CYAN;   lab = "AM";  break;
        case DSP_MODE_NFM: fg = COL_YELLOW; lab = "NFM"; break;
        default:           fg = COL_GREEN;  lab = "WFM"; break;
        }
        /* Show a box only while touched, so the idle header stays clean while
         * still being obviously tappable once you press it. */
        if (s_mode_hot) {
            rect(MODE_BOX_X0, MODE_BOX_Y0, MODE_BOX_X1 - MODE_BOX_X0,
                 MODE_BOX_Y1 - MODE_BOX_Y0, fg);
            fg = COL_BG;
        }
        text_c((MODE_BOX_X0 + MODE_BOX_X1) / 2, y + 4, 2, fg, lab);
    }

    draw_scan_buttons();

    /* --- row 3: parameter strip ------------------------------------------ */
    int yy = 50;
    int px = 8;
    snprintf(b, sizeof b, "FPS %2d", s_fps);
    text(px, yy, 1, COL_TEXT, b);

    px += 62;
    snprintf(b, sizeof b, "REF %+d", (int)lroundf(s_ref_db));
    text(px, yy, 1, COL_TEXT, b);

    px += 68;
    snprintf(b, sizeof b, "DROP %uK", (unsigned)(st.dropped_bytes / 1024));
    text(px, yy, 1, st.dropped_bytes ? COL_RED : COL_DIM, b);

    /* live peak of the current frame, in the same nominal dBm scale */
    snprintf(b, sizeof b, "PK %+6.1f", s_peak_dbm);
    text_r(632, yy, 1, COL_GREEN, b);

    hline(0, HDR_H - 1, LW, COL_GRID);
}

/* ---- spectrum ------------------------------------------------------------ */

static void draw_graticule(void)
{
    /* horizontal: one line per 10 dB row, 8 rows across the plot */
    for (int i = 0; i <= GRID_DIV; i++) {
        int y = SPEC_Y0 + (i * SPEC_H) / GRID_DIV;
        hline(PLOT_X0, y, PLOT_W, (i == 0 || i == GRID_DIV) ? COL_GRID : COL_GRIDV);
    }
    /* vertical: dotted columns, like the analyzer's fine graticule */
    for (int i = 0; i <= GRID_DIV; i++) {
        int x = PLOT_X0 + (i * PLOT_W) / GRID_DIV;
        for (int y = SPEC_Y0; y < SPEC_Y0 + SPEC_H; y += 3)
            LBUF[(size_t)y * LW + x] = COL_GRIDV;
    }

    /* dBm labels on both edges — 10 dB per row */
    char b[12];
    for (int i = 0; i <= GRID_DIV; i++) {
        int y = SPEC_Y0 + (i * SPEC_H) / GRID_DIV - 3;
        snprintf(b, sizeof b, "%d", (int)lroundf(s_ref_db) - i * 10);
        text_r(PLOT_X0 - 6, y, 1, COL_DIM, b);
        text(PLOT_X1 + 6, y, 1, COL_DIM, b);
    }
}

static void draw_ruler(void)
{
    /* Clear the whole band between the plot and the waterfall first.
     *
     * draw_spectrum() only clears SPEC_Y0..SPEC_Y0+SPEC_H, which stops above
     * this band, and text() draws set pixels without a background. So without
     * this the labels from the previous frequency stay behind and the new ones
     * land on top of them — visibly ghosted text while dragging to retune. */
    rect(0, SPEC_Y0 + SPEC_H, LW, WF_Y0 - (SPEC_Y0 + SPEC_H), COL_BG);

    /* Frequency ruler: one tick + label per vertical gridline, labelled in
     * MHz like the analyzer head in the reference (left edge -> right edge). */
    float span = SDR_DEFAULT_SAMPLE_RATE / 2.0f;
    char b[16];

    hline(PLOT_X0, RULER_Y, PLOT_W, COL_GRID);
    for (int i = 0; i <= GRID_DIV; i++) {
        int x = PLOT_X0 + (i * PLOT_W) / GRID_DIV;
        vline(x, RULER_Y, 4, COL_GRID);

        float hz = (float)s_center - span + 2.0f * span * (float)i / GRID_DIV;
        snprintf(b, sizeof b, "%.3f", hz / 1e6);
        int tx = x - text_w(b, 1) / 2;
        if (tx < 2) tx = 2;
        if (tx + text_w(b, 1) > LW - 2) tx = LW - 2 - text_w(b, 1);
        text(tx, RULER_Y + 6, 1, (i == GRID_DIV / 2) ? COL_CURSOR : COL_DIM, b);
    }
}

static void draw_spectrum(const float *spec, int bins)
{
    /* ---- reference level: follow the frame's own noise floor ------------- */
    float lo = spec[0];
    for (int i = 1; i < bins; i++)
        if (spec[i] < lo) lo = spec[i];
    s_nf_dbm = lo;
    float target = lo + DB_SPAN; /* noise sits on the bottom */
    if (!s_ref_init) { s_ref_db = target; s_ref_init = true; }
    else s_ref_db += (target - s_ref_db) * REF_SLEW;

    /* clear plot + gutters, then the graticule under the trace */
    rect(0, SPEC_Y0, LW, SPEC_H, COL_BG);
    draw_graticule();

    /* ---- trace: fill below the curve, bright 1px curve on top ----------- */
    float hi = spec[0];
    for (int i = 0; i < PLOT_W; i++) {
        int bin = (i * bins) / PLOT_W;
        float db = spec[bin];
        if (db > hi) hi = db;
        int ytop = level_to_y(db);
        int x = PLOT_X0 + i;
        for (int y = ytop; y < SPEC_Y0 + SPEC_H; y++)
            LBUF[(size_t)y * LW + x] = COL_FILL;
        LBUF[(size_t)ytop * LW + x] = COL_TRACE;

        /* peak hold: instant attack, slow decay */
        float dbm = db;
        if (dbm > s_peak[i] || s_peak[i] == 0) s_peak[i] = dbm;
        else s_peak[i] -= PEAK_DECAY;
        float pt = (s_ref_db - s_peak[i]) / DB_SPAN;
        int py = SPEC_Y0 + (int)(pt * SPEC_H);
        if (py >= SPEC_Y0 && py < SPEC_Y0 + SPEC_H)
            LBUF[(size_t)py * LW + x] = COL_PEAK;
    }
    s_peak_dbm = hi;

    /* centre cursor */
    vline(LW / 2, SPEC_Y0, SPEC_H, RGB565(60, 30, 30));

    draw_ruler();

    /* ---- waterfall: scroll one row down, push the newest frame on top --- */
    /* Rows are LW pixels wide but only the PLOT_W-wide plot area scrolls, so
     * this has to be a per-row copy — a single memmove of a contiguous
     * (WF_H-1)*PLOT_W block would walk diagonally through the buffer. */
    for (int y = WF_Y0 + WF_H - 1; y > WF_Y0; y--)
        memcpy(LBUF + (size_t)y * LW + PLOT_X0,
               LBUF + (size_t)(y - 1) * LW + PLOT_X0,
               (size_t)PLOT_W * sizeof(uint16_t));
    for (int i = 0; i < PLOT_W; i++) {
        int bin = (i * bins) / PLOT_W;
        /* Waterfall has its own, much tighter range: the noise floor maps to
         * WF_BASE (deep blue, not black) and ~25 dB above it is white. */
        float over = spec[bin] - s_nf_dbm;
        float t = WF_BASE + over / WF_SPAN;
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        LBUF[(size_t)WF_Y0 * LW + PLOT_X0 + i] = s_wlut[(int)(t * 255.0f)];
    }

    /* waterfall time axis in the left gutter (cleared so labels don't smear) */
    rect(0, WF_Y0, PLOT_X0, WF_H, COL_BG);
    static const int MARKS = 6;
    for (int i = 0; i <= MARKS; i++) {
        int y = WF_Y0 + (i * (WF_H - 1)) / MARKS;
        hline(PLOT_X0 - 4, y, 4, COL_DIM);
        char tb[10];
        float sec = (float)(i * (WF_H - 1) / MARKS) / (s_fps > 0 ? s_fps : 20);
        snprintf(tb, sizeof tb, "-%.1f", sec);
        text_r(PLOT_X0 - 7, y - 3, 1, COL_DIM, tb);
    }
}

/* ---- volume -------------------------------------------------------------- */

/* Vertical 8-step level bar in the gutter to the right of the waterfall.
 *
 * Nothing else paints in this strip: draw_spectrum() only scrolls PLOT_W
 * columns, and the dBm labels stop at the bottom of the spectrum band. So the
 * control occupies dead space instead of competing with the plot. */
static void draw_volume(void)
{
    const int x0 = VOL_ZONE_X0, w = VOL_ZONE_X1 - VOL_ZONE_X0;
    const int y0 = VOL_BAR_TOP - 16, y1 = VOL_BAR_BOT + 2;
    rect(x0, y0, w, y1 - y0, COL_BG);

    const int gap = 2;
    const int span = VOL_BAR_BOT - VOL_BAR_TOP;
    const int seg_h = (span - (VOL_SEGS - 1) * gap) / VOL_SEGS;
    const int bw = VOL_BAR_X1 - VOL_BAR_X0;

    /* Segments are stacked bottom-up, so a taller bar reads as "louder". */
    for (int i = 0; i < VOL_SEGS; i++) {
        int yb = VOL_BAR_BOT - i * (seg_h + gap);
        int yt = yb - seg_h;
        bool on = (s_vol_level > i);
        uint16_t c;
        if (!on) c = COL_GRIDV;
        else if (s_vol_hot) c = COL_CYAN;
        else if (i < VOL_SEGS / 2) c = COL_GREEN;
        else c = COL_TRACE; /* upper half brighter: reads as "loud" */
        rect(VOL_BAR_X0, yt, bw, seg_h, c);
    }

    /* "VOL" label above the bar. The bar itself already shows the level, so a
     * number would be redundant — the label is what the reader actually needs.
     * It is only 18 px wide (3 glyphs x 6), which fits the 32 px gutter. */
    text_c((x0 + VOL_ZONE_X1) / 2, VOL_BAR_TOP - 13, 1,
           s_vol_hot ? COL_CYAN : COL_DIM, "VOL");
}

/* Map a touch Y inside the bar to a level. Tapping a segment selects it, so
 * the whole 8-step range is reachable with single taps. */
static int vol_level_from_y(int ly)
{
    int span = VOL_BAR_BOT - VOL_BAR_TOP;
    if (span <= 0) return s_vol_level;
    int up = VOL_BAR_BOT - ly; /* 0 at the bottom, span at the top */
    if (up < 0) up = 0;
    if (up > span) up = span;
    /* Round to the nearest segment so a tap selects the one under the finger. */
    int lvl = (up * VOL_SEGS + span / 2) / span;
    if (lvl < 1) lvl = 1;
    if (lvl > VOL_SEGS) lvl = VOL_SEGS;
    return lvl;
}

static void set_vol_level(int lvl)
{
    if (lvl < 1) lvl = 1;
    if (lvl > VOL_SEGS) lvl = VOL_SEGS;
    if (lvl == s_vol_level) return;
    s_vol_level = lvl;
    /* The codec takes 0..100; spread the 8 steps across the full range. */
    audio_set_volume(lvl * 100 / VOL_SEGS);
    s_redraw = true;
}

/* ---- numeric keypad ------------------------------------------------------ */
/*
 * A modal frequency entry panel. Tapping the 7-segment readout opens it.
 *
 * Frequencies are entered in MHz with an optional decimal point ("121.5"),
 * because that is how they are actually quoted; typing nine raw Hz digits would
 * be slow and easy to get wrong by a factor of ten. The panel is seeded with the
 * current frequency so a one-digit edit does not mean retyping everything.
 *
 * While open it is modal: every touch is consumed here, and it is painted last
 * (after the live spectrum) so nothing can draw over it.
 */
#define KP_CANCEL 12
#define KP_OK     13
#define KP_NONE   (-1)

/* Digits the user may type. Kept well below sizeof s_kp_buf, which has to be
 * roomy because snprintf("%.6g") is allowed to emit exponent form. */
#define KP_MAX_LEN 10

static void kp_key_rect(int idx, int *x, int *y)
{
    *x = KP_X0 + KP_PAD + (idx % 3) * (KP_KEY_W + KP_GAP);
    *y = KP_GRID_Y + (idx / 3) * (KP_KEY_H + KP_GAP);
}

/* Which control is at (lx,ly)? Key index 0..11, KP_CANCEL, KP_OK or KP_NONE. */
static int kp_hit(int lx, int ly)
{
    for (int i = 0; i < 12; i++) {
        int x, y;
        kp_key_rect(i, &x, &y);
        if (lx >= x && lx < x + KP_KEY_W && ly >= y && ly < y + KP_KEY_H) return i;
    }
    int by = KP_BTN_Y;
    if (ly >= by && ly < by + KP_BTN_H) {
        if (lx >= KP_X0 + KP_PAD && lx < KP_X0 + KP_PAD + KP_BTN_W) return KP_CANCEL;
        if (lx >= KP_X0 + KP_PAD + KP_BTN_W + KP_GAP &&
            lx < KP_X0 + KP_PAD + 2 * KP_BTN_W + KP_GAP) return KP_OK;
    }
    return KP_NONE;
}

static void kp_open(void)
{
    /* %.6g keeps it compact: 100 -> "100", 121.5 -> "121.5", 145.125 -> "145.125".
     * "%.3f" would pad to "121.500", which then needs three deletes to edit. */
    snprintf(s_kp_buf, sizeof s_kp_buf, "%.6g", s_center / 1e6);
    s_kp_len = (int)strlen(s_kp_buf);
    s_kp_open = true;
    s_kp_hot = -1;
}

static void kp_close(bool apply)
{
    if (apply) {
        char *end = NULL;
        double mhz = strtod(s_kp_buf, &end);
        if (end != s_kp_buf && mhz > 0) {
            double hz = mhz * 1e6;
            /* Clamp rather than reject: typing "1215" means 1215 MHz, which is
             * out of range, and landing on the band edge is more useful than
             * silently doing nothing. */
            if (hz < FREQ_MIN) hz = FREQ_MIN;
            if (hz > FREQ_MAX) hz = FREQ_MAX;
            ui_set_freq((uint32_t)(hz + 0.5));
        }
    }
    s_kp_open = false;
    s_kp_hot = -1;
    s_kp_closed = true; /* tell ui_task to repaint what was underneath */
}

static void kp_apply(int hit)
{
    if (hit >= 0 && hit < 12) {
        const char *lab = KP_LABELS[hit];
        if (!strcmp(lab, "DEL")) {
            if (s_kp_len > 0) s_kp_buf[--s_kp_len] = '\0';
        } else if (!strcmp(lab, ".")) {
            /* One decimal point only, and never as the first character. */
            if (s_kp_len > 0 && !strchr(s_kp_buf, '.') &&
                s_kp_len < KP_MAX_LEN) {
                s_kp_buf[s_kp_len++] = '.';
                s_kp_buf[s_kp_len] = '\0';
            }
        } else if (s_kp_len < KP_MAX_LEN) {
            s_kp_buf[s_kp_len++] = lab[0];
            s_kp_buf[s_kp_len] = '\0';
        }
    } else if (hit == KP_CANCEL) {
        kp_close(false);
    } else if (hit == KP_OK) {
        kp_close(true);
    }
}

static void draw_keypad(void)
{
    /* Replace the whole buffer instead of dimming it. The waterfall scrolls in
     * place inside this same buffer, so an overlay would be pushed downwards as
     * if it were history and leave ghost rows behind. Painting black here and
     * rebuilding everything on close keeps the two consistent. */
    rect(0, 0, LW, LH, COL_BG);
    rect(KP_X0, KP_Y0, KP_W, KP_H, COL_BG);
    hline(KP_X0, KP_Y0, KP_W, COL_CYAN);
    hline(KP_X0, KP_Y0 + KP_H - 1, KP_W, COL_CYAN);
    vline(KP_X0, KP_Y0, KP_H, COL_CYAN);
    vline(KP_X0 + KP_W - 1, KP_Y0, KP_H, COL_CYAN);

    /* --- entered value --------------------------------------------------- */
    rect(KP_X0 + KP_PAD, KP_VAL_Y, KP_W - 2 * KP_PAD, KP_VAL_H, RGB565(0, 26, 14));
    const char *shown = s_kp_len ? s_kp_buf : "_";
    text(KP_X0 + KP_PAD + 6, KP_VAL_Y + 12, 2, COL_WHITE, shown);
    text_r(KP_X0 + KP_W - KP_PAD - 6, KP_VAL_Y + 8, 1, COL_DIM, "MHz");

    /* --- keys ------------------------------------------------------------ */
    for (int i = 0; i < 12; i++) {
        int x, y;
        kp_key_rect(i, &x, &y);
        bool hot = (s_kp_hot == i);
        rect(x, y, KP_KEY_W, KP_KEY_H, hot ? COL_GREEN : RGB565(0, 34, 18));
        hline(x, y, KP_KEY_W, hot ? COL_GREEN : COL_GRID);
        text_c(x + KP_KEY_W / 2, y + KP_KEY_H / 2 - 7, 2,
               hot ? COL_BG : COL_GREEN, KP_LABELS[i]);
    }

    /* --- buttons --------------------------------------------------------- */
    int by = KP_BTN_Y;
    int cx = KP_X0 + KP_PAD;
    bool hc = (s_kp_hot == KP_CANCEL), ho = (s_kp_hot == KP_OK);
    rect(cx, by, KP_BTN_W, KP_BTN_H, hc ? COL_RED : RGB565(40, 12, 12));
    text_c(cx + KP_BTN_W / 2, by + KP_BTN_H / 2 - 7, 2,
           hc ? COL_BG : COL_RED, "CANCEL");
    int ox = cx + KP_BTN_W + KP_GAP;
    rect(ox, by, KP_BTN_W, KP_BTN_H, ho ? COL_GREEN : RGB565(10, 40, 20));
    text_c(ox + KP_BTN_W / 2, by + KP_BTN_H / 2 - 7, 2,
           ho ? COL_BG : COL_GREEN, "OK");
}

/* ---- footer -------------------------------------------------------------- */

static void draw_footer(void)
{
    rect(0, FOOT_Y, LW, LH - FOOT_Y, COL_BG);
    hline(0, FOOT_Y, LW, COL_GRID);
    text(8, FOOT_Y + 5, 1, COL_DIM, "TAB5-SA  RTL2832U");
    text_c(LW / 2, FOOT_Y + 5, 1, COL_DIM, "SPECTRUM / WATERFALL");
    text_r(632, FOOT_Y + 5, 1, COL_DIM, "TUNE:TAP/DRAG  MODE:TAP  VOL:RIGHT");
}

/* ---- present ------------------------------------------------------------- */

static void present(void)
{
    /* 90° maps the landscape buffer onto the portrait panel upright; 270° is the
     * same rotation the other way, which turns the whole image upside down.
     * Both produce 360x640 before the 2x scale, so the output geometry is
     * identical and nothing else has to change. */
    ppa_do_scale_rotate_mirror(s_ppa, &(ppa_srm_oper_config_t){
        .in = { .buffer = LBUF, .pic_w = LW, .pic_h = LH, .block_w = LW, .block_h = LH,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .out = { .buffer = bsp_tab5_display_get_frame_buffer(s_fb_index),
                 .buffer_size = PANEL_W * PANEL_H * 2, .pic_w = PANEL_W, .pic_h = PANEL_H,
                 .srm_cm = PPA_SRM_COLOR_MODE_RGB565 },
        .rotation_angle = s_flipped ? PPA_SRM_ROTATION_ANGLE_270
                                    : PPA_SRM_ROTATION_ANGLE_90,
        .scale_x = SCALE, .scale_y = SCALE,
    });
    bsp_tab5_display_flush(s_fb_index);
    s_fb_index ^= 1;
}

/* Static "receiver stopped" picture.
 *
 * Same layout as an idle receiver, but blanked on purpose: the plot and the
 * waterfall are the two things that are only meaningful while IQ is flowing, so
 * leaving the last frame on screen would look like a live but silent band. The
 * graticule and ruler stay, because they describe the tuning, not the signal. */
static void draw_rx_off(void)
{
    const int y0 = SPEC_Y0;
    const int h  = (WF_Y0 + WF_H) - y0;

    rect(0, y0, LW, h, COL_BG);
    draw_graticule();
    draw_ruler();

    int cy = WF_Y0 + WF_H / 2;
    text_c(LW / 2, cy - 12, 2, COL_YELLOW, "STOPPED");
    text_c(LW / 2, cy + 12, 1, COL_DIM, "press play to resume");

    draw_header();
    draw_volume();
    draw_footer();
}

/* ---- touch --------------------------------------------------------------- */

/* Tuning sensitivity while dragging. The visible span is only
 * SDR_DEFAULT_SAMPLE_RATE/2 = 576 kHz across a 574 px plot, i.e. ~2 kHz per
 * pixel would be a strict 1:1 "grab". That feels sluggish on a 5" panel, so
 * 4 kHz/px gives a full-width swipe ~2.5 MHz (about two visible spans) while
 * still resolving fine moves: 200 kHz of FM dial spacing is a 50 px drag. */
#define DRAG_HZ_PER_PX 4000

/* Retuning the R820T costs several USB control transfers (the RTL2832 bridges
 * them to the tuner's I2C), so pushing every pixel of a drag would hammer the
 * same USB link the IQ stream depends on. Coalesce: only push once the target
 * has moved by at least this much (2.5 px at the sensitivity above). */
#define MIN_TUNE_STEP_HZ 10000

/* Retune without dropping the spectrum accumulator. ui_set_freq() flushes the
 * half-mixed frame, which is right for discrete jumps but would blank the
 * display if done on every frame of a continuous drag. */
static void tune_live(int64_t hz)
{
    if (hz < FREQ_MIN) hz = FREQ_MIN;
    if (hz > FREQ_MAX) hz = FREQ_MAX;
    if ((uint32_t)hz == s_center) return;
    s_center = (uint32_t)hz;
    sdr_set_center_freq(s_center);
    s_redraw = true; /* repaint the frequency readout on the very next frame */
}

/* A horizontal move smaller than this (logical px, so ~16 px on the glass) is
 * treated as finger jitter, not as a drag. Without it every tap would retune by
 * a few kHz before the finger came up. */
#define TAP_SLOP 8

/* Tapping a point in the plot retunes so that point becomes the centre — i.e.
 * whatever is under the finger moves to the middle of the screen. The scale is
 * the visible span (SDR_DEFAULT_SAMPLE_RATE/2 either side of centre), not the
 * drag sensitivity, so the tap lands exactly where it was aimed. */
static void tune_to_plot_x(int lx)
{
    const float span = SDR_DEFAULT_SAMPLE_RATE / 2.0f; /* half-width, Hz */
    const float hz_per_px = 2.0f * span / (float)PLOT_W;

    /* Clamp to the plot: a tap in the dBm label gutter (x < PLOT_X0) or past the
     * right edge should land on the nearest edge of the span, not extrapolate
     * into a frequency that is not on screen. */
    if (lx < PLOT_X0) lx = PLOT_X0;
    if (lx > PLOT_X1) lx = PLOT_X1;

    const float off = (float)(lx - (PLOT_X0 + PLOT_W / 2)) * hz_per_px;
    int64_t want = (int64_t)llroundf((float)s_center + off);

    if (want < FREQ_MIN) want = FREQ_MIN;
    if (want > FREQ_MAX) want = FREQ_MAX;
    /* Discrete jump, so drop the half-mixed accumulator like ui_set_freq(). */
    ui_set_freq((uint32_t)want);
}

static void toggle_mode(void)
{
    /* WFM -> NFM -> AM -> WFM. One badge, one tap: no hidden gestures, and the
     * badge always shows the mode that is actually running. */
    s_mode = (s_mode + 1) % DSP_MODE_COUNT;
    dsp_set_mode(s_mode);
    s_redraw = true;
}

/* ---- scan state machine -------------------------------------------------- */

static void scan_stop(void)
{
    s_scan_dir = 0;
    s_scan_left = 0;
    s_redraw = true;
}

static void scan_start(int dir)
{
    s_scan_dir = dir;
    s_scan_left = SEEK_MAX_STEPS;
    s_scan_next = esp_timer_get_time(); /* step on the next tick */
    s_redraw = true;
}

/*
 * Advance the hunt. Called from the UI loop; does nothing unless an arrow is
 * active and its tick interval has elapsed.
 *
 * The step is slow on purpose (10 per second): the tuner needs several control
 * transfers to retune and the demodulator has to settle before the level at the
 * new frequency means anything. Stepping faster would judge stale data.
 */
static void scan_tick(void)
{
    if (s_scan_dir == 0) return;

    int64_t now = esp_timer_get_time();
    if (now < s_scan_next) return;

    /* Judge where we are BEFORE moving on: the tick interval is what gives the
     * reading time to become valid. */
    if (dsp_signal_db() > dsp_noise_floor_db() + SCAN_HOLD_DB) {
        scan_stop(); /* found something: park here */
        return;
    }

    if (--s_scan_left <= 0) {
        scan_stop(); /* reached the end of the hunt without finding anything */
        return;
    }

    int64_t want = (int64_t)s_center + (int64_t)s_scan_dir * SCAN_STEP_HZ;
    if (want > (int64_t)FREQ_MAX) { scan_stop(); return; } /* band edge */
    if (want < (int64_t)FREQ_MIN) { scan_stop(); return; }

    tune_live(want);
    s_scan_next = now + SCAN_TICK_US;
}

/* Read the panel and apply gestures. Returns true when the screen needs a
 * repaint. */
static bool handle_touch(void)
{
    static bool prev, dragging, moved;
    static int drag_x0, tap_x, run, grace;
    static uint32_t drag_f0;
    bool changed = false;
    esp_lcd_touch_point_data_t p;
    int n = bsp_tab5_touch_read(&p, 1);

    /* The controller reports stray points for ~1 s after init. Without this
     * grace period the mode badge toggles itself at boot. */
    if (grace < 150) { grace++; prev = false; run = 0; return false; }

    /* A press must survive 2 polls (~16 ms) before it counts: rejects
     * single-sample glitches without making taps feel laggy. */
    run = (n > 0) ? run + 1 : 0;
    bool pressed = run >= 2;

    int lx = 0, ly = 0;
    if (pressed) {
        /* The panel is portrait 720x1280; this buffer is 640x360, which the PPA
         * scaled 2x and rotated 90°. Undo both to get landscape coordinates.
         * When the image is flipped the rotation is 270° instead, so the
         * inverse mapping is mirrored too — otherwise every touch would land
         * on the opposite side of the screen. */
        if (s_flipped) {
            lx = p.y / SCALE;
            ly = (PANEL_W - p.x) / SCALE;
        } else {
            lx = (PANEL_H - p.y) / SCALE;
            ly = p.x / SCALE;
        }
        if (lx < 0) lx = 0;
        if (lx >= LW) lx = LW - 1;
        if (ly < 0) ly = 0;
        if (ly >= LH) ly = LH - 1;
    }
    bool edge = pressed && !prev;

    /* While the keypad is open it is modal: every touch belongs to it, and the
     * gestures underneath (tune, volume, mode) must not fire. Acting on release
     * rather than on press means a finger dragged onto the wrong key can be
     * slid off before letting go, like a real keypad. */
    if (s_kp_open) {
        if (!pressed) {
            int done = s_kp_hot;
            s_kp_hot = KP_NONE;
            if (done != KP_NONE) { kp_apply(done); changed = true; }
        } else {
            int hit = kp_hit(lx, ly);
            if (hit != s_kp_hot) { s_kp_hot = hit; changed = true; }
        }
        prev = pressed;
        return changed;
    }

    if (!pressed) {
        /* A press that ended without ever moving past TAP_SLOP is a tap: retune
         * so the touched point becomes the centre. `dragging` is only set by the
         * plot branch, so this can never fire for the badge or the volume bar. */
        if (dragging && !moved) {
            tune_to_plot_x(tap_x);
            changed = true;
        }
        dragging = false;
        moved = false;
        if (s_mode_hot) {
            s_mode_hot = false;
            /* Short tap switches demod; a long press re-learns "upright" from
             * the way the device is being held right now. */
            if (s_mode_hold >= MODE_LONG_POLLS) {
                bsp_tab5_orientation_set_upright();
                bsp_tab5_orient_t o;
                bsp_tab5_orientation_get(&o);
                s_flipped = o.flipped;
            } else {
                toggle_mode();
            }
            s_mode_hold = 0;
            changed = true;
        }
        if (s_vol_hot)  { s_vol_hot  = false; changed = true; }
        if (s_scan_hot >= 0) { s_scan_hot = -1; changed = true; }
    }

    if (edge && scan_hit(lx, ly) >= 0) {
        /* Scan transport. Acted on press rather than release: these are
         * latched actions (start/stop), not a value the finger is dragging, so
         * there is nothing to fine-tune by sliding. */
        int b = scan_hit(lx, ly);
        if (b == 1) {
            /* Play/stop is a real run/stop, not a mute: stopping releases the
             * dongle, so no IQ is read at all and the screen goes to the same
             * static "no data" state as an unplugged receiver. */
            bool on = !sdr_get_running();
            sdr_set_running(on);
            if (!on) {
                /* Drop the half-accumulated frame so the frozen picture cannot
                 * be mistaken for a live one. The repaint is only flagged here,
                 * NOT done here: ui_task fetches the spectrum pointer before it
                 * calls handle_touch(), so painting now would be immediately
                 * overwritten by draw_spectrum() using that stale pointer —
                 * leaving one full frame frozen on screen forever. */
                dsp_flush_spectrum();
                s_rx_off_paint = true;
            }
            s_redraw = true;
        } else {
            /* Arrows hunt in one direction and park on the first signal. */
            int dir = (b == 0) ? -1 : +1;
            if (s_scan_dir == dir) scan_stop();
            else                   scan_start(dir);
        }
        s_scan_hot = b;
        dragging = false;
        changed = true;
    } else if (edge && lx >= MODE_BOX_X0 && lx < MODE_BOX_X1 &&
        ly >= MODE_BOX_Y0 && ly < MODE_BOX_Y1) {
        /* Mode badge: act on RELEASE, so a hold can mean something different
         * from a tap (see the release branch). */
        s_mode_hot = true;
        s_mode_hold = 0;
        dragging = false;
        changed = true;
    } else if (pressed && s_mode_hot) {
        s_mode_hold++;
    } else if (edge && lx >= FREQ_BOX_X0 && lx < FREQ_BOX_X1 &&
               ly >= FREQ_BOX_Y0 && ly < FREQ_BOX_Y1) {
        /* Tap the big 7-segment readout to type a frequency directly. */
        kp_open();
        dragging = false;
        changed = true;
    } else if (pressed && lx >= VOL_ZONE_X0 &&
               ly >= VOL_BAR_TOP - 16 && ly <= VOL_BAR_BOT + 2) {
        /* Volume gutter. Tested BEFORE the waterfall branch: the gutter lies
         * inside the same vertical band, and that branch keys off ly only, so
         * it would otherwise swallow every touch on the bar.
         *
         * The y range matters: the right-hand dBm labels sit at the same x
         * inside the spectrum band, and without this check tapping a label
         * would silently change the volume instead of doing nothing. */
        int lvl = vol_level_from_y(ly);
        if (lvl != s_vol_level) { set_vol_level(lvl); changed = true; }
        if (!s_vol_hot) { s_vol_hot = true; changed = true; }
        dragging = false;
    } else if (pressed && ly >= SPEC_Y0 && ly < WF_Y0 + WF_H) {
        /* Spectrum or waterfall: drag to slide the frequency, or tap to jump. */
        if (edge) {
            scan_stop(); /* a manual tune always takes back control */
            dragging = true;
            moved = false;
            drag_x0 = lx;
            tap_x = lx;
            drag_f0 = s_center;
            changed = true;
        } else if (dragging && lx != drag_x0) {
            /* Do not tune until the finger has clearly committed to a drag.
             * Otherwise a slightly shaky tap would retune by a few kHz and then
             * the release would jump again — two conflicting actions. */
            if (!moved && (lx > drag_x0 ? lx - drag_x0 : drag_x0 - lx) >= TAP_SLOP) {
                moved = true;
                drag_x0 = lx;      /* re-baseline so crossing the slop is smooth */
                drag_f0 = s_center;
            }
            if (!moved) goto touch_done;

            /* Pulling the trace to the right means the receiver moves DOWN in
             * frequency, so the drag delta is subtracted. */
            int64_t want = (int64_t)drag_f0 - (int64_t)(lx - drag_x0) * DRAG_HZ_PER_PX;
            int64_t delta = want - (int64_t)s_center;
            if (delta < 0) delta = -delta;
            if (delta >= MIN_TUNE_STEP_HZ) {
                tune_live(want);
                changed = true;
            }
        }
    } else if (edge) {
        dragging = false;
    }

touch_done:
    prev = pressed;
    return changed;
}

static void ui_task(void *arg)
{
    (void)arg;
    rect(0, 0, LW, LH, COL_BG);
    rect(PLOT_X0, WF_Y0, PLOT_W, WF_H, COL_BG);
    draw_header();
    draw_volume();
    draw_footer();
    draw_graticule();
    present();

    const float *last = NULL;
    int bins = dsp_spectrum_bins();
    int stale = 0;        /* loops since the last new spectrum frame  */
    int hdr_div = 0;      /* header refresh divider (~4 Hz)           */
    int frames = 0;       /* frames counted in the current second     */
    int64_t sec0 = esp_timer_get_time();
    int idle_repaint = 0; /* keeps the no-dongle screen alive         */
    int64_t bat_next = 0; /* next battery poll (I2C is not free)      */
    int64_t orient_next = 0; /* next IMU poll (shares the I2C bus)    */
    bool no_shown = false;

    while (1) {
        /* Battery: the INA226 needs a couple of I2C transactions, so poll it on
         * a wall-clock schedule instead of once per frame. */
        if (esp_timer_get_time() >= bat_next) {
            bat_next = esp_timer_get_time() + 2000000; /* every 2 s */
            bsp_tab5_battery_read(&s_bat);
            s_redraw = true;
        }

        /* Orientation: 10 Hz is plenty for a screen flip and keeps I2C traffic
         * (shared with the touch controller and the IMU itself) low. The poll
         * returns true only when the flip state actually changed, so a stable
         * device causes no repaints. */
        if (esp_timer_get_time() >= orient_next) {
            orient_next = esp_timer_get_time() + 100000; /* every 100 ms */
            if (bsp_tab5_orientation_poll()) {
                bsp_tab5_orient_t o;
                bsp_tab5_orientation_get(&o);
                s_flipped = o.flipped;
                /* The buffer content is still valid; only the mapping onto the
                 * panel changed, so a plain present() is enough. */
                present();
            }
        }

        /* ---- modal keypad ------------------------------------------------ */
        /* While it is open nothing else may draw: the waterfall scrolls in place
         * inside this same buffer, so a live update would push the panel pixels
         * downwards as if they were history. */
        if (s_kp_open) {
            static bool painted;
            bool ch = handle_touch(); /* may close the panel via OK / CANCEL */
            if (s_kp_open) {
                if (ch || !painted) { draw_keypad(); present(); painted = true; }
            } else {
                painted = false; /* closed: rebuild the live screen next pass */
            }
            vTaskDelay(pdMS_TO_TICKS(8));
            continue;
        }

        /* Panel just closed: it had painted over the whole buffer, so rebuild
         * the static furniture. The next spectrum frame refills the plot. */
        if (s_kp_closed) {
            s_kp_closed = false;
            rect(0, 0, LW, LH, COL_BG);
            rect(PLOT_X0, WF_Y0, PLOT_W, WF_H, COL_BG);
            draw_graticule();
            draw_ruler();
            draw_header();
            draw_volume();
            draw_footer();
            present();
        }

        /* Scan transport. Self-gating on a wall-clock interval: it steps at
         * 10 Hz regardless of how fast this loop runs, so the tuner and the
         * demodulator get time to settle between steps. */
        scan_tick();

        const float *spec = dsp_spectrum();
        bool newspec = spec && spec != last;
        bool touched = handle_touch();

        if (newspec) {
            last = spec;
            stale = 0;
            frames++;
            if (esp_timer_get_time() - sec0 >= 1000000) {
                s_fps = frames;
                frames = 0;
                sec0 = esp_timer_get_time();
                s_redraw = true;
            }
            draw_spectrum(spec, bins);
            /* The header carries live readings (SIG, FPS, DROP, REF) and must
             * agree with the dBm labels just painted — repaint it when the
             * rounded reference level moves, or every 5 frames otherwise. */
            int ref_now = (int)lroundf(s_ref_db);
            if (s_redraw || ref_now != s_ref_shown || ++hdr_div >= 5) {
                s_redraw = false;
                s_ref_shown = ref_now;
                hdr_div = 0;
                draw_header();
            }
            if (no_shown) { no_shown = false; draw_footer(); }
        } else if (stale < 1000) {
            stale++;
        }

        /* No spectrum for >~0.5 s. That is the NORMAL state when the dongle is
         * absent — this port is shared with the flashing/serial link. Report
         * it plainly and keep the header ticking so the screen is obviously
         * alive rather than frozen. */
        if (stale > 62) {
            if (!no_shown || ++idle_repaint >= 62) {
                no_shown = true;
                idle_repaint = 0;
                draw_header();
                if (!sdr_get_running()) {
                    /* Deliberately stopped: this is not a fault, so do not
                     * blame the cable. */
                    text_c(LW / 2, WF_Y0 + WF_H / 2, 2, COL_YELLOW, "STOPPED");
                } else if (!sdr_is_present()) {
                    text_c(LW / 2, WF_Y0 + WF_H / 2 - 12, 2, COL_YELLOW, "NO RTL-SDR");
                    text_c(LW / 2, WF_Y0 + WF_H / 2 + 12, 1, COL_DIM,
                           "plug the dongle into the USB-A port");
                } else {
                    text_c(LW / 2, WF_Y0 + WF_H / 2, 2, COL_RED, "NO DATA - CHECK USB");
                }
                draw_footer();
                present();
            }
        }

        /* Stopped just now: blank the plot and waterfall once, after any frame
         * that was already in flight has been drawn. Doing this inside
         * handle_touch() would lose the race against draw_spectrum(). */
        if (s_rx_off_paint) {
            s_rx_off_paint = false;
            draw_rx_off();
            present();
        }

        /* A gesture can change the mode badge, the tuned frequency or the
         * volume. This must NOT be gated on !newspec: while the dongle is
         * streaming, almost every iteration produces a new spectrum frame, so
         * a volume drag would never repaint. Header and volume strip are cheap
         * (they do not touch the waterfall), so redraw them on any touch. */
        if (touched) {
            draw_header();
            draw_volume();
        }

        if (newspec || touched) present();
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

void ui_start(void)
{
    LBUF = heap_caps_malloc((size_t)LW * LH * 2, MALLOC_CAP_SPIRAM);
    if (!LBUF) { ESP_LOGE(TAG, "no PSRAM for landscape buffer"); return; }
    ESP_ERROR_CHECK(ppa_register_client(&(ppa_client_config_t){ .oper_type = PPA_OPERATION_SRM }, &s_ppa));
    build_wlut();

    sdr_stats_t st;
    sdr_get_stats(&st);
    s_center = st.center_freq;
    s_mode = dsp_get_mode();
    audio_set_volume(s_vol_level * 100 / VOL_SEGS);

    ESP_LOGI(TAG, "spectrum console %dx%d (PPA x%d, display-only)", LW, LH, SCALE);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 6, NULL, 0);
}

/* ---- external control hooks (no touch panel in this build) ---------------- */

void ui_set_freq(uint32_t hz)
{
    if (hz < FREQ_MIN) hz = FREQ_MIN;
    if (hz > FREQ_MAX) hz = FREQ_MAX;
    s_center = hz;
    sdr_set_center_freq(hz);
    dsp_flush_spectrum(); /* drop the half-mixed frame from the old frequency */
    s_redraw = true;
}

uint32_t ui_get_freq(void) { return s_center; }

void ui_set_mode(int mode)
{
    if (mode < DSP_MODE_WFM || mode >= DSP_MODE_COUNT) mode = DSP_MODE_WFM;
    s_mode = mode;
    dsp_set_mode(s_mode);
    s_redraw = true;
}

void ui_set_volume(int vol)
{
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;
    /* Map the 0..100 range onto the 8 discrete steps. */
    set_vol_level((vol * VOL_SEGS + 50) / 100);
    s_redraw = true;
}
