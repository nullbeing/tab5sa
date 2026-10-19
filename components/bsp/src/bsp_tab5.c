/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Hiroki Kawakami
 */

#include "bsp_private.h"
#include "bsp_tab5.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_timer.h"
#include "esp_codec_dev_defaults.h"
#include "es8388_codec.h"
#include "misc/bsp_display.h"
#include "pi4io/pi4io.h"
#include "ina226/ina226.h"
#include "bmi270/bmi270.h"
#include "ili9881c/ili9881c.h"
#include "gt911/gt911.h"
#include "st7123/st7123_lcd.h"
#include "st7123/st7123_touch.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_hosted.h"
#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_hosted_bt.h"
#endif
#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#endif

static const char *TAG = "BSP_TAB5";

#define I2C0_PORT_NUM (0)
static i2c_master_bus_handle_t i2c0;
static pi4io_t pi4ioe1, pi4ioe2;

typedef enum {
    BSP_PANEL_ILI9881C,
    BSP_PANEL_ST7123,
} bsp_panel_type_t;

static void **frame_buffers;
static ili9881c_lcd_t ili9881c;
static gt911_touch_t gt911;
static st7123_lcd_t st7123_lcd;
static st7123_touch_t st7123_touch;

/* Battery monitor (U31 INA226 across the 5 mOhm shunt R39). Optional: stays
 * NULL when the part is absent or does not answer. */
static ina226_t ina226;
static bool charge_enabled = true;

/* Orientation (U19 BMI270). Optional: stays NULL when the part is absent. */
static bmi270_t bmi270;
static bool orient_flipped;               /* current reported state        */
static bool orient_ref_valid;             /* upright direction known yet?  */
static int32_t orient_ref_x, orient_ref_y; /* in-plane gravity when upright */
static int16_t orient_ax, orient_ay, orient_az;

/* Touch controller interrupt / GT911 address-strap pin. */
#define TOUCH_INT_GPIO GPIO_NUM_23

/*
 * Reset the touch controller and work out which I2C address it latched.
 *
 * The GT911 decides its address from the level of INT at the rising edge of
 * TP_RST: low -> 0x5D, high -> 0x14. `esp_lcd_touch_gt911` implements exactly
 * that sequence, but only when it is given a reset GPIO. On the Tab5 TP_RST
 * lives on the PI4IOE expander, so the driver is handed GPIO_NUM_NC and skips
 * the whole block — leaving INT to `gpio_reset_pin()`, i.e. floating on the
 * internal pull-up.
 *
 * That is a genuine race: whichever level the pin happens to sit at when RST
 * releases wins, and it depends on how fast the rails came up. It showed up as
 * "touch dead after a cold boot, fine after a reset". Driving INT low makes the
 * latch deterministic (0x5D).
 *
 * Retried because the part also needs time to boot before it answers at all;
 * a single probe on a slow power-up would otherwise look like "no touch".
 */
static uint8_t touch_reset_and_probe(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        /* Hold INT low across the reset so the address strap reads as 0x5D. */
        gpio_config(&(gpio_config_t){
            .pin_bit_mask = 1ULL << TOUCH_INT_GPIO,
            .mode = GPIO_MODE_OUTPUT,
        });
        gpio_set_level(TOUCH_INT_GPIO, 0);

        pi4io_set_output(pi4ioe1, 5, false); /* TP_RST = Low */
        vTaskDelay(pdMS_TO_TICKS(20));
        pi4io_set_output(pi4ioe1, 5, true);  /* TP_RST = High: address latched */
        vTaskDelay(pdMS_TO_TICKS(120));      /* GT911 needs >100 ms to boot */

        /* Hand INT back to the input path the interrupt handler expects.
         * Keep the pull-up: `gpio_reset_pin()` used to leave one enabled, and
         * the INT line is open-drain, so without it the edge never arrives. */
        gpio_config(&(gpio_config_t){
            .pin_bit_mask = 1ULL << TOUCH_INT_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        });

        if (i2c_master_probe(i2c0, GT911_I2C_ADDR_PRIMARY, 50) == ESP_OK)
            return GT911_I2C_ADDR_PRIMARY;
        if (i2c_master_probe(i2c0, GT911_I2C_ADDR_BACKUP, 50) == ESP_OK)
            return GT911_I2C_ADDR_BACKUP;

        ESP_LOGW(TAG, "touch: no GT911 after reset (attempt %d)", attempt + 1);
    }
    return 0;
}

esp_err_t bsp_tab5_init(const bsp_tab5_config_t *config) {
    esp_err_t err;

    // Check config values
    bsp_tab5_config_t tmp_config = *config;
    if (!tmp_config.display.fb_num) tmp_config.display.fb_num = 1;
    config = &tmp_config;

    // Initialize I2C0 bus
    err = i2c_new_master_bus(&(i2c_master_bus_config_t){
        .i2c_port = I2C0_PORT_NUM,
        .sda_io_num = GPIO_NUM_31,
        .scl_io_num = GPIO_NUM_32,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    }, &i2c0);
    BSP_RETURN_ERR(err);

    // Initialize PI4IOE1 (address 0x43)
    err = pi4io_init(i2c0, 0x43, (pi4io_pin_config_t[8]){
        [0] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = false },  // RF_INT_EXT_SWITCH
        [1] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // SPK_EN
        [2] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // EXT5V_EN
        [4] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // LCD_RST
        [5] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // TP_RST
        [6] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // CAM_RST
        [7] = { PI4IO_PIN_MODE_INPUT },                           // HP_DET
    }, &pi4ioe1);
    BSP_RETURN_ERR(err);

    // Initialize PI4IOE2 (address 0x44)
    err = pi4io_init(i2c0, 0x44, (pi4io_pin_config_t[8]){
        [0] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // WLAN_PWR_EN
        [3] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = false },  // USB5V_EN
        [4] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = false },  // PWROFF_PLUSE
        [5] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = false },  // nCHG_QC_EN
        [6] = { PI4IO_PIN_MODE_INPUT,  .pull = PI4IO_PULL_DOWN },  // CHG_STAT
        // CHG_EN must be HIGH from boot. Unlike a plain ESP32 dev board, the
        // Tab5 charges its 2S pack through an IP2326 whose enable line is
        // driven from here — leaving it low means the device never charges,
        // however long it stays plugged in.
        [7] = { PI4IO_PIN_MODE_OUTPUT, .initial_value = true },   // CHG_EN
    }, &pi4ioe2);
    BSP_RETURN_ERR(err);
    charge_enabled = true;

    /* Battery monitor: probe the INA226. A missing part is not fatal — the UI
     * simply reports no battery data. */
    if (ina226_init(i2c0, &ina226) != ESP_OK) {
        ESP_LOGW(TAG, "no INA226 battery monitor; voltage/current unavailable");
    }

    /* Orientation: non-fatal. Without an IMU the screen simply never flips. */
    if (bsp_tab5_orientation_init() != ESP_OK) {
        ESP_LOGW(TAG, "no BMI270; screen flip disabled");
    }

    /* Reset the LCD. Touch reset is handled separately: the GT911 needs its INT
     * pin driven while TP_RST is released (see touch_reset_and_probe). */
    pi4io_set_output(pi4ioe1, 4, false);  // LCD_RST = Low
    vTaskDelay(pdMS_TO_TICKS(100));
    pi4io_set_output(pi4ioe1, 4, true);   // LCD_RST = High
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * Panel/touch detection.
     *
     * The panel sits on MIPI-DSI, which is write-only here: it has no I2C
     * presence and no ID we can read back. Every DSI init command "succeeds"
     * whether or not the panel understood it, so a wrong panel driver produces
     * a black screen with a completely clean log. The only signal available is
     * which touch controller answers on I2C, and the panel is inferred from
     * that pairing. Log the raw probe results so a bad inference is visible
     * from the serial output instead of having to be guessed at.
     */
    bool have_st7123_tp = (i2c_master_probe(i2c0, ST7123_TOUCH_I2C_ADDR, 50) == ESP_OK);

    /* Pulses TP_RST, which also resets the ST7123 if that is the fitted part. */
    uint8_t gt911_addr = touch_reset_and_probe();
    if (!gt911_addr) {
        /* The ST7123 does not use the INT strap; re-probe in case the pulse
         * above brought it up. */
        have_st7123_tp = (i2c_master_probe(i2c0, ST7123_TOUCH_I2C_ADDR, 50) == ESP_OK);
    }

    ESP_LOGI(TAG, "touch probe: st7123@0x%02x=%s gt911@0x%02x=%s gt911@0x%02x=%s",
             ST7123_TOUCH_I2C_ADDR, have_st7123_tp ? "yes" : "no",
             GT911_I2C_ADDR_PRIMARY, gt911_addr == GT911_I2C_ADDR_PRIMARY ? "yes" : "no",
             GT911_I2C_ADDR_BACKUP, gt911_addr == GT911_I2C_ADDR_BACKUP ? "yes" : "no");

    if (!have_st7123_tp && !gt911_addr) {
        ESP_LOGE(TAG, "No touch controller found on I2C0 — cannot identify board revision");
        return ESP_ERR_NOT_FOUND;
    }

    bsp_panel_type_t panel = have_st7123_tp ? BSP_PANEL_ST7123 : BSP_PANEL_ILI9881C;
#if defined(CONFIG_BSP_TAB5_PANEL_FORCE_ILI9881C)
    panel = BSP_PANEL_ILI9881C;
    ESP_LOGW(TAG, "panel driver forced to ILI9881C by Kconfig");
#elif defined(CONFIG_BSP_TAB5_PANEL_FORCE_ST7123)
    panel = BSP_PANEL_ST7123;
    ESP_LOGW(TAG, "panel driver forced to ST7123 by Kconfig");
#endif
    ESP_LOGI(TAG, "panel driver: %s (inferred from touch; override in menuconfig "
                  "if the screen stays black)",
             panel == BSP_PANEL_ST7123 ? "ST7123" : "ILI9881C");

    // Initialize the panel
    if (panel == BSP_PANEL_ST7123) {
        err = st7123_lcd_init(&(st7123_lcd_config_t){
            .backlight_gpio = GPIO_NUM_22,
            .size = (bsp_size_t){ 720, 1280 },
            .pixel_format = BSP_PIXEL_FORMAT_RGB565,
            .fb_num = config->display.fb_num,
        }, &st7123_lcd);
        BSP_RETURN_ERR(err);
        frame_buffers = st7123_lcd_get_frame_buffers(st7123_lcd);
    } else {
        err = ili9881c_lcd_init(&(ili9881c_lcd_config_t){
            .backlight_gpio = GPIO_NUM_22,
            .size = (bsp_size_t){ 720, 1280 },
            .pixel_format = BSP_PIXEL_FORMAT_RGB565,
            .fb_num = config->display.fb_num,
        }, &ili9881c);
        BSP_RETURN_ERR(err);
        frame_buffers = ili9881c_lcd_get_frame_buffers(ili9881c);
    }

    // Initialize the touch controller that actually answered, independent of
    // the panel choice above — the two are separate parts.
    if (have_st7123_tp) {
        err = st7123_touch_init(&(st7123_touch_config_t){
            .i2c_bus = i2c0,
            .size = (bsp_size_t){ 720, 1280 },
            .int_gpio = TOUCH_INT_GPIO,
            .rst_gpio = GPIO_NUM_NC,
            .scl_speed_hz = 100000,
            .interrupt = config->touch.interrupt,
        }, &st7123_touch);
        BSP_RETURN_ERR(err);
    } else {
        err = gt911_touch_init(&(gt911_touch_config_t){
            .i2c_bus = i2c0,
            .size = (bsp_size_t){ 720, 1280 },
            .int_gpio = TOUCH_INT_GPIO,
            .rst_gpio = GPIO_NUM_NC,
            .scl_speed_hz = 100000,
            .interrupt = config->touch.interrupt,
            .i2c_addr = gt911_addr,
        }, &gt911);
        BSP_RETURN_ERR(err);
    }

    if (config->wifi.enable || config->bluetooth.enable) {
        // NVS (for WiFi & Bluetooth)
        err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            if ((err = nvs_flash_erase()) == ESP_OK) {
                err = nvs_flash_init();
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize NVS flash");
            return err;
        }
    }

    // WiFi
    if (config->wifi.enable) {
        ESP_LOGE(TAG, "WiFi initialization not implemented yet!");
        assert(0);
        // ESP_ERROR_CHECK(esp_netif_init());
        // ESP_ERROR_CHECK(esp_event_loop_create_default());
        // esp_netif_create_default_wifi_ap();
        // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    }

    // Bluetooth
    if (config->bluetooth.enable) {
#if defined(CONFIG_BT_BLUEDROID_ENABLED)
        /* initialize TRANSPORT first */
        hosted_hci_bluedroid_open();

        /* get HCI driver operations */
        esp_bluedroid_hci_driver_operations_t operations = {
            .send = hosted_hci_bluedroid_send,
            .check_send_available = hosted_hci_bluedroid_check_send_available,
            .register_host_callback = hosted_hci_bluedroid_register_host_callback,
        };
        esp_bluedroid_attach_hci_driver(&operations);
#elif defined(CONFIG_BT_NIMBLE_ENABLED)
        err = nimble_port_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize NimBLE");
            return err;
        }
#else
        ESP_LOGE(TAG, "Bluetooth Stack is not Enabled.");
#endif
    }

    return ESP_OK;
}

// MARK: Audio (ES8388 speaker DAC over I2S)
// Pins per the M5Stack Tab5 schematic; ES8388 sits on the shared I2C0 bus, and
// its power amp is gated by SPK_EN (PI4IOE1 P1), already enabled in bsp_tab5_init.
#define BSP_I2S_MCLK GPIO_NUM_30
#define BSP_I2S_BCLK GPIO_NUM_27
#define BSP_I2S_WS   GPIO_NUM_29
#define BSP_I2S_DOUT GPIO_NUM_26
#define BSP_I2S_DIN  GPIO_NUM_28

static i2s_chan_handle_t i2s_tx;

esp_codec_dev_handle_t bsp_tab5_audio_speaker_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan_cfg, &i2s_tx, NULL) != ESP_OK) return NULL;

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK, .bclk = BSP_I2S_BCLK, .ws = BSP_I2S_WS,
            .dout = BSP_I2S_DOUT, .din = BSP_I2S_DIN,
        },
    };
    if (i2s_channel_init_std_mode(i2s_tx, &std_cfg) != ESP_OK) return NULL;

    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&(audio_codec_i2s_cfg_t){
        .port = I2S_NUM_0,
        .tx_handle = i2s_tx,
    });
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&(audio_codec_i2c_cfg_t){
        .port = I2C0_PORT_NUM,
        .addr = ES8388_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c0,
    });
    const audio_codec_if_t *es8388 = es8388_codec_new(&(es8388_codec_cfg_t){
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,   /* P4 I2S is master, codec is slave */
        .ctrl_if     = ctrl_if,
        .pa_pin      = -1,      /* PA handled by SPK_EN, not the codec */
    });
    if (!data_if || !ctrl_if || !es8388) return NULL;

    return esp_codec_dev_new(&(esp_codec_dev_cfg_t){
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8388,
        .data_if  = data_if,
    });
}

// MARK: USB host power
void bsp_tab5_set_usb_host_power(bool on) {
    // PI4IOE2 (0x44) pin 3 = USB5V_EN — gates 5V VBUS on the USB-A host port.
    pi4io_set_output(pi4ioe2, 3, on);
}

// MARK: Battery / charging
void bsp_tab5_set_charge_enable(bool on) {
    // PI4IOE2 (0x44) pin 7 = CHG_EN -> IP2326 enable.
    if (!pi4ioe2) return;
    if (pi4io_set_output(pi4ioe2, 7, on) == ESP_OK) charge_enabled = on;
}

bool bsp_tab5_charge_enabled(void) { return charge_enabled; }

/* 2S Li-ion pack: map pack voltage to a rough state of charge. The curve is
 * the usual resting-voltage approximation — it is not coulomb counting, so it
 * sags under load and reads low while the dongle is drawing current. */
static int pack_percent(int32_t mv)
{
    static const struct { int32_t mv; int pct; } CURVE[] = {
        {8400, 100}, {8200, 92}, {8000, 85}, {7800, 70}, {7600, 55},
        {7400, 40},  {7200, 25}, {7000, 12}, {6800, 5},  {6400, 0},
    };
    const int n = BSP_ARRAY_SIZE(CURVE);
    if (mv >= CURVE[0].mv) return 100;
    if (mv <= CURVE[n - 1].mv) return 0;
    for (int i = 0; i < n - 1; i++) {
        if (mv <= CURVE[i].mv && mv >= CURVE[i + 1].mv) {
            int32_t hi = CURVE[i].mv, lo = CURVE[i + 1].mv;
            int phi = CURVE[i].pct, plo = CURVE[i + 1].pct;
            int32_t span = hi - lo;
            if (span <= 0) return plo;
            return plo + (int)((int32_t)(phi - plo) * (mv - lo) / span);
        }
    }
    return 0;
}

void bsp_tab5_battery_read(bsp_tab5_battery_t *out) {
    if (!out) return;
    memset(out, 0, sizeof *out);

    bool external = false;
    if (pi4ioe2) {
        bool stat = false;
        /* PI4IOE2 bit6 is the IP2326 BAT_STAT line. Per the IP2326 datasheet it
         * is LOW while trickle charging and HIGH once constant-current charging
         * begins, so HIGH means "the charger is working". The pin is configured
         * with a pull-DOWN (matching the M5Stack BSP, which also pulls P6 down),
         * so an idle or terminated charger reads LOW instead of floating high. */
        if (pi4io_get_input(pi4ioe2, 6, &stat) == ESP_OK) external = stat;
    }
    out->external = external;

    if (!ina226) {
        out->present = false;
        return;
    }

    int32_t mv = 0;
    float ma = 0;
    if (ina226_bus_mv(ina226, &mv) != ESP_OK) return;
    /* A failed current read must NOT be reported as 0 mA: that looks like a
     * perfectly idle pack and hides an I2C fault. Keep present=false instead. */
    if (ina226_current_ma(ina226, &ma) != ESP_OK) return;

    out->present = true;
    out->mv = mv;
    out->ma = ma;
    out->percent = pack_percent(mv);
    /* Charging is decided by the IP2326 status line, NOT by the shunt current.
     * In the constant-voltage phase the current tapers towards zero and then
     * terminates, so a pack at 97% draws ~0 mA while it is still being charged.
     * The old `abs_ma > 30` test therefore reported "discharging" for exactly
     * the case this indicator exists for — measured on hardware as
     * `bat 8.33V +0mA 97% dischg` with the charger connected. */
    out->charging = external && charge_enabled;
}

void bsp_tab5_battery_dump(void) {
    uint8_t in = 0, out = 0;
    if (pi4ioe2) {
        pi4io_get_all_inputs(pi4ioe2, &in);
        /* OUT_SET read back from the chip, not the cached value: proves the
         * CHG_EN write actually landed instead of silently failing. */
        pi4io_get_all_outputs(pi4ioe2, &out);
    }
    ESP_LOGI(TAG, "pi4ioe2 in=0x%02X (bit6 CHG_STAT=%d) out=0x%02X (bit7 CHG_EN=%d, want %d)",
             in, (in >> 6) & 1, out, (out >> 7) & 1, (int)charge_enabled);
    ina226_dump(ina226);
}

// MARK: Display
void bsp_tab5_display_set_brightness(int brightness) {
    if (ili9881c) ili9881c_lcd_set_brightness(ili9881c, brightness);
    if (st7123_lcd) st7123_lcd_set_brightness(st7123_lcd, brightness);
}
void *bsp_tab5_display_get_frame_buffer(int fb_index) {
    return frame_buffers[fb_index];
}
void bsp_tab5_display_flush(int fb_index) {
    if (ili9881c) ili9881c_lcd_flush(ili9881c, fb_index);
    if (st7123_lcd) st7123_lcd_flush(st7123_lcd, fb_index);
}

// MARK: Touch Panel
int bsp_tab5_touch_read(esp_lcd_touch_point_data_t *points, uint8_t max_points) {
    if (gt911) return gt911_touch_read(gt911, points, max_points);
    if (st7123_touch) return st7123_touch_read(st7123_touch, points, max_points);
    return 0;
}
void bsp_tab5_touch_wait_interrupt(void) {
    if (gt911) gt911_touch_wait_interrupt(gt911);
    if (st7123_touch) st7123_touch_wait_interrupt(st7123_touch);
}

// MARK: Orientation (BMI270)
/*
 * "Upside down" is judged from gravity *inside the plane of the screen*.
 *
 * Held in landscape the panel is roughly vertical, so the in-plane component
 * (x,y) points towards the floor. Turning the device over in its own plane
 * reverses that vector — which is exactly the "screen is upside down" case.
 * Comparing a direction rather than one hard-coded axis means no assumption
 * about how the sensor is mounted: whatever orientation the device was in when
 * the reference was learned is taken as upright, and everything is relative.
 *
 * Lying flat on a desk the in-plane component is ~0, and gravity then carries no
 * information about in-plane rotation. That is physically undetectable, so the
 * last state is held instead of being guessed from noise.
 */
#define ORIENT_SMOOTH   4    /* EMA divisor for the samples                */
#define ORIENT_MIN_MG   300  /* in-plane gravity needed to decide anything */
/* Learning the reference needs a much larger in-plane component than merely
 * acting on it: when the panel is near flat the in-plane vector is small and
 * its *direction* is mostly noise, so latching a reference from it would make
 * later comparisons meaningless. Require the device to be genuinely held up
 * (which puts ~1 g in plane) before trusting a direction. */
#define ORIENT_LATCH_MG 600

/* A direction is only adopted after it has held steady for about a second.
 *
 * Magnitude alone is not enough: while the device is being turned over, the
 * in-plane component stays near 1 g the whole way while its *direction* sweeps
 * through every angle. Latching during that sweep can capture a direction that
 * is upside down, which leaves the flip inverted for the rest of the session.
 * Requiring the direction to stay put is what makes the sample trustworthy. */
#define ORIENT_STABLE_POLLS 10   /* at the 100 ms poll rate            */
#define ORIENT_STABLE_COS2  9409 /* cos(14 deg)^2, scaled by 10000     */

/* The upright direction is a property of how the board is mounted, so it is
 * learned once and stored. Re-learning it on every boot turned the flip into a
 * coin toss: whichever orientation the device happened to be in at that moment
 * became "upright" for the whole session. */
#define ORIENT_NVS_NAMESPACE "orient"
#define ORIENT_NVS_KEY_X     "ref_x"
#define ORIENT_NVS_KEY_Y     "ref_y"

static int32_t orient_ema[3];
static int32_t orient_cand_x, orient_cand_y; /* direction being evaluated */
static int orient_cand_n;                    /* consecutive stable polls  */
static int16_t orient_rx, orient_ry, orient_rz;

static void orient_read_raw(int16_t *x, int16_t *y, int16_t *z)
{
    *x = *y = *z = 0;
    if (bmi270) {
        bmi270_accel_mg(bmi270, x, y, z);
        bmi270_accel_raw(bmi270, &orient_rx, &orient_ry, &orient_rz);
    }
    orient_ax = *x;
    orient_ay = *y;
    orient_az = *z;
}

static bool orient_inplane_significant(int32_t x, int32_t y)
{
    return x * x + y * y >= ORIENT_MIN_MG * ORIENT_MIN_MG;
}

static bool orient_inplane_strong(int32_t x, int32_t y)
{
    return x * x + y * y >= ORIENT_LATCH_MG * ORIENT_LATCH_MG;
}

/* NVS is only initialised on demand: the BSP does it for WiFi/BT, and this build
 * enables neither, but the orientation reference still needs a home. */
static bool orient_nvs_ready(void)
{
    static int state; /* 0 = untried, 1 = ok, -1 = unavailable */
    if (state) return state > 0;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() == ESP_OK) err = nvs_flash_init();
    }
    state = (err == ESP_OK) ? 1 : -1;
    if (state < 0)
        ESP_LOGW(TAG, "NVS unavailable; orientation will be re-learned each boot");
    return state > 0;
}

static bool orient_ref_load(void)
{
    if (!orient_nvs_ready()) return false;

    nvs_handle_t h;
    if (nvs_open(ORIENT_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    int32_t x = 0, y = 0;
    esp_err_t ex = nvs_get_i32(h, ORIENT_NVS_KEY_X, &x);
    esp_err_t ey = nvs_get_i32(h, ORIENT_NVS_KEY_Y, &y);
    nvs_close(h);

    if (ex != ESP_OK || ey != ESP_OK) return false;
    if (!orient_inplane_strong(x, y)) return false; /* stale or corrupt */

    orient_ref_x = x;
    orient_ref_y = y;
    orient_ref_valid = true;
    return true;
}

static void orient_ref_save(int32_t x, int32_t y)
{
    if (!orient_nvs_ready()) return;

    nvs_handle_t h;
    if (nvs_open(ORIENT_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_i32(h, ORIENT_NVS_KEY_X, x) == ESP_OK &&
        nvs_set_i32(h, ORIENT_NVS_KEY_Y, y) == ESP_OK)
        nvs_commit(h);
    nvs_close(h);
}

/* Adopt `x,y` as the upright direction and remember it. */
static void orient_ref_adopt(int32_t x, int32_t y)
{
    orient_ref_x = x;
    orient_ref_y = y;
    orient_ref_valid = true;
    orient_cand_n = 0;
    orient_flipped = false;
    orient_ref_save(x, y);
    ESP_LOGI(TAG, "orientation reference %+ld/%+ld mg (upright)", (long)x, (long)y);
}

esp_err_t bsp_tab5_orientation_init(void)
{
    if (bmi270_init(i2c0, &bmi270) != ESP_OK) return ESP_ERR_NOT_FOUND;

    /* Average a few samples: one reading is noisy and would bias the reference
     * for the rest of the session. */
    int32_t sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < 8; i++) {
        int16_t x, y, z;
        orient_read_raw(&x, &y, &z);
        sx += x; sy += y; sz += z;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    orient_ema[0] = sx / 8;
    orient_ema[1] = sy / 8;
    orient_ema[2] = sz / 8;

    /* Do not latch here. Even a strong reading says nothing about whether the
     * device is being moved at this instant; the poll loop learns it once the
     * direction has been stable for a while. Until then the screen stays
     * upright, which is the harmless default. */
    orient_flipped = false;

    if (orient_ref_load()) {
        ESP_LOGI(TAG, "orientation reference restored %+ld/%+ld mg",
                 (long)orient_ref_x, (long)orient_ref_y);
    } else {
        ESP_LOGI(TAG, "no stored orientation; will learn once held still");
    }
    return ESP_OK;
}

bool bsp_tab5_orientation_poll(void)
{
    if (!bmi270) {
        /* A boot whose 8 KiB config upload failed would otherwise leave the
         * screen flip dead until the next reset. Retry a few times in the
         * background so the feature heals itself. Deliberately bounded: each
         * attempt blocks for ~1 s (the upload), so this must not run forever. */
        static int64_t next_try;
        static int tries;
        int64_t now = esp_timer_get_time();
        if (tries < 3 && now >= next_try) {
            tries++;
            next_try = now + 8000000; /* 8 s apart */
            if (bsp_tab5_orientation_init() == ESP_OK) {
                ESP_LOGI(TAG, "IMU recovered on background retry %d", tries);
            }
        }
        return false;
    }

    int16_t x, y, z;
    orient_read_raw(&x, &y, &z);
    if (x == 0 && y == 0 && z == 0) return false; /* no sample yet */

    orient_ema[0] += ((int32_t)x - orient_ema[0]) / ORIENT_SMOOTH;
    orient_ema[1] += ((int32_t)y - orient_ema[1]) / ORIENT_SMOOTH;
    orient_ema[2] += ((int32_t)z - orient_ema[2]) / ORIENT_SMOOTH;

    int32_t cx = orient_ema[0], cy = orient_ema[1];

    /* Flat on a desk: no in-plane gravity, so no in-plane orientation. */
    if (!orient_inplane_significant(cx, cy)) return false;

    if (!orient_ref_valid) {
        /* Learn only from a direction that is both strong and steady. */
        if (!orient_inplane_strong(cx, cy)) {
            orient_cand_n = 0;
            return false;
        }

        if (orient_cand_n == 0) {
            orient_cand_x = cx;
            orient_cand_y = cy;
            orient_cand_n = 1;
            return false;
        }

        /* Still pointing the same way as the candidate? cos^2 > cos(14deg)^2. */
        int64_t d  = (int64_t)cx * orient_cand_x + (int64_t)cy * orient_cand_y;
        int64_t tt = (int64_t)(cx * cx + cy * cy) *
                     (int64_t)(orient_cand_x * orient_cand_x + orient_cand_y * orient_cand_y);
        if (d > 0 && 10000 * d * d > (int64_t)ORIENT_STABLE_COS2 * tt) {
            orient_cand_n++;
            /* Track the candidate as a running average so it settles exactly
             * on the resting direction rather than on the first sample. */
            orient_cand_x += (cx - orient_cand_x) / 4;
            orient_cand_y += (cy - orient_cand_y) / 4;
        } else {
            orient_cand_x = cx;
            orient_cand_y = cy;
            orient_cand_n = 1;
        }

        if (orient_cand_n >= ORIENT_STABLE_POLLS)
            orient_ref_adopt(orient_cand_x, orient_cand_y);
        return false;
    }

    int32_t r2 = orient_ref_x * orient_ref_x + orient_ref_y * orient_ref_y;
    int64_t dot = (int64_t)cx * orient_ref_x + (int64_t)cy * orient_ref_y;
    int64_t tot = (int64_t)(cx * cx + cy * cy) * r2; /* |cur|^2 * |ref|^2 */

    bool was = orient_flipped;
    /* cos(angle) = dot / sqrt(tot).
     *   flip   once the direction is >120 deg away (dot < 0 and
     *          4*dot^2 > tot, i.e. cos < -0.5)
     *   unflip as soon as it comes back within 90 deg (dot > 0)
     * The gap between those two thresholds is the dead band that keeps the
     * screen from flickering while the device is tilted through the middle. */
    if (!orient_flipped) {
        if (dot < 0 && 4 * dot * dot > tot) orient_flipped = true;
    } else {
        if (dot > 0) orient_flipped = false;
    }

    return orient_flipped != was;
}

bool bsp_tab5_orientation_set_upright(void)
{
    if (!bmi270) return false;

    int32_t cx = orient_ema[0], cy = orient_ema[1];
    if (!orient_inplane_strong(cx, cy)) {
        ESP_LOGW(TAG, "cannot set upright: only %+ld/%+ld mg in plane (too flat)",
                 (long)cx, (long)cy);
        return false;
    }

    bool changed = orient_flipped;
    orient_ref_adopt(cx, cy);
    ESP_LOGI(TAG, "upright set manually from the current orientation");
    return changed;
}

void bsp_tab5_orientation_get(bsp_tab5_orient_t *out)
{
    if (!out) return;
    out->present = (bmi270 != NULL);
    out->flipped = orient_flipped;
    out->ax = orient_ax;
    out->ay = orient_ay;
    out->az = orient_az;
    out->rx = orient_rx;
    out->ry = orient_ry;
    out->rz = orient_rz;
    out->range = bmi270_accel_range(bmi270);
}
