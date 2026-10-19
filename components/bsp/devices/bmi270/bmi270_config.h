/* The Bosch configuration blob (~8 KiB) is required before the BMI270 produces
 * any data. It is copied verbatim from the esp-dsp M5Stack example. */
#pragma once
#include <stdint.h>

extern const uint8_t bmi270_config[];
extern const int bmi270_config_size;
