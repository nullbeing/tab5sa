/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Hiroki Kawakami
 */

#pragma once
#include "bsp_common.h"
#include "esp_lcd_touch.h"
#include "esp_codec_dev.h"

typedef struct {
    struct {
        uint8_t fb_num;
    } display;
    struct {
        bool interrupt;
    } touch;
    struct {
        bool enable;
    } wifi;
    struct {
        bool enable;
    } bluetooth;
} bsp_tab5_config_t;

esp_err_t bsp_tab5_init(const bsp_tab5_config_t *config);
// Enable/disable the 5V VBUS supplied to the USB-A host port (off by default).
// Required for host-mode peripherals such as an RTL-SDR dongle.
void bsp_tab5_set_usb_host_power(bool on);

/*
 * Battery / charging.
 *
 * Unlike a bare ESP32 dev board, the Tab5 does NOT charge just because USB is
 * plugged in: the 2S pack is charged by an IP2326 whose enable line (CHG_EN) is
 * driven by the PI4IOE5V6408 I/O expander. bsp_tab5_init() asserts it so the
 * device accepts charging from boot, and it stays enabled until explicitly
 * turned off. Charge state comes back on CHG_STAT (open-drain, LOW = charging)
 * and the pack voltage/current from the INA226 across the 5 mOhm shunt R39.
 */
typedef struct {
    bool present;    /* INA226 answered on the I2C bus            */
    int32_t mv;      /* pack voltage in millivolts (2S)           */
    float ma;        /* current: >0 into the pack (charging)      */
    int percent;     /* 0..100, from the 2S discharge curve       */
    bool charging;   /* CHG_STAT asserted (LOW)                   */
    bool external;   /* external power likely present             */
} bsp_tab5_battery_t;

/* Snapshot battery state. Safe to call from any task, any time; returns zeros
 * with present=false when no INA226 is fitted. */
void bsp_tab5_battery_read(bsp_tab5_battery_t *out);

/* Log the INA226 register map raw (diagnostics only). No-op without a device. */
void bsp_tab5_battery_dump(void);

/* Assert / release the IP2326 charge enable. */
void bsp_tab5_set_charge_enable(bool on);
bool bsp_tab5_charge_enabled(void);

/*
 * Orientation (BMI270 accelerometer).
 *
 * The Tab5 has a 6-axis IMU (U19, BMI270) on the same I2C bus. Gravity gives an
 * absolute "which way is down" reference, so the UI can flip itself when the
 * device is turned upside down. The gyro is deliberately not used: integrating
 * angular rate to get orientation drifts.
 *
 * There is no documented axis mapping for this board, so bsp_tab5_orientation_read()
 * does not assume one. Instead it reports the gravity component on whichever axis
 * currently dominates, signed relative to the orientation captured at boot. That
 * makes "upside down" a pure sign flip, independent of how the part is mounted.
 */
typedef struct {
    bool present;    /* a BMI270 answered and is producing samples */
    bool flipped;    /* gravity points opposite to the boot reference */
    int16_t ax, ay, az; /* latest sample, milli-g (diagnostics) */
    int16_t rx, ry, rz; /* latest sample, raw counts (calibration) */
    uint8_t range;   /* ACC_RANGE in effect: 0=+-2g 1=+-4g 2=+-8g 3=+-16g */
} bsp_tab5_orient_t;

/* Configure the IMU and latch the boot orientation as "upright".
 * Non-fatal: without an IMU the UI simply never flips. */
esp_err_t bsp_tab5_orientation_init(void);

/* Poll the IMU and update the flipped flag (with hysteresis so noise near the
 * horizontal position cannot make the screen flicker). Returns true when the
 * flip state changed. Safe to call when no IMU is present. */
bool bsp_tab5_orientation_poll(void);

/* Adopt the device's CURRENT orientation as upright and store it.
 *
 * The reference is normally learned automatically, but a rare bad latch would
 * otherwise leave the flip inverted forever with no way back. Returns true when
 * the on-screen flip state changed as a result. */
bool bsp_tab5_orientation_set_upright(void);

/* Last known orientation state. */
void bsp_tab5_orientation_get(bsp_tab5_orient_t *out);
// Initialise the ES8388 speaker DAC and return an esp_codec_dev output handle.
esp_codec_dev_handle_t bsp_tab5_audio_speaker_init(void);
void bsp_tab5_display_set_brightness(int brightness);
void *bsp_tab5_display_get_frame_buffer(int fb_index);
void bsp_tab5_display_flush(int fb_index);
int bsp_tab5_touch_read(esp_lcd_touch_point_data_t *points, uint8_t max_points);
void bsp_tab5_touch_wait_interrupt(void);
