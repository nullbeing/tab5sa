/*
 * bmi270.h — Bosch BMI270 6-axis IMU (accelerometer used for screen flip).
 *
 * The Tab5 fits a BMI270 (U19) on the system I2C bus, alongside the touch
 * controller and the audio codecs. Only the accelerometer is used here: gravity
 * gives an absolute "which way is down" reference, whereas the gyro measures
 * angular rate and would need integration (and drift) to produce an orientation.
 *
 * Bosch's part needs its ~8 KiB configuration blob uploaded before it produces
 * any data, so bmi270_init() does a soft reset, uploads the blob and enables the
 * accelerometer. That takes a few hundred ms at 400 kHz.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct bmi270_state *bmi270_t;

/* Candidate addresses: 0x68 when SDO is tied low, 0x69 when tied high. The
 * strap is not documented for the Tab5, so both are probed. */
#define BMI270_ADDR_LOW  0x68
#define BMI270_ADDR_HIGH 0x69

/* Configure the accelerometer at +-4 g / 100 Hz.
 * Returns ESP_ERR_NOT_FOUND when no BMI270 answers with a valid chip ID. */
esp_err_t bmi270_init(i2c_master_bus_handle_t bus, bmi270_t *out);

esp_err_t bmi270_deinit(bmi270_t dev);

/* Address the device was found at (for logging). */
uint8_t bmi270_address(bmi270_t dev);

/* One accelerometer sample in milli-g (1 g = 1000). Axes are the sensor's own;
 * their orientation relative to the panel is determined empirically, not by
 * assumption. */
esp_err_t bmi270_accel_mg(bmi270_t dev, int16_t *x, int16_t *y, int16_t *z);

/* Raw signed counts straight from the data registers, plus the ACC_RANGE
 * setting actually in effect. Used for calibration/diagnostics: comparing raw
 * counts against the known 1 g magnitude is the only way to verify that the
 * range register matches the sensitivity the driver assumes. */
esp_err_t bmi270_accel_raw(bmi270_t dev, int16_t *x, int16_t *y, int16_t *z);
uint8_t bmi270_accel_range(bmi270_t dev);
