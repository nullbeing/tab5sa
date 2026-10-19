/*
 * ina226.h — TI INA226 bidirectional current/power monitor.
 *
 * Tab5 uses one (U31) across the 5 mOhm shunt R39 to measure the 2S battery
 * pack: bus voltage = pack voltage, current = charge (+) / discharge (-).
 *
 * The I2C address is set by the A0/A1 straps and is not printed on the
 * schematic, so ina226_init() probes the address range and validates the
 * manufacturer/die IDs before accepting a device.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct ina226_state *ina226_t;

/* Shunt resistor on Tab5 (R39). */
#define INA226_SHUNT_OHMS 0.005f

/* Probe `bus` for an INA226 and configure it for continuous bus+shunt
 * conversion. Returns ESP_OK and a handle on success, ESP_ERR_NOT_FOUND when
 * no device in the candidate range answers with valid IDs. */
esp_err_t ina226_init(i2c_master_bus_handle_t bus, ina226_t *out);

/* Release the I2C device (the bus itself is owned by the caller). */
esp_err_t ina226_deinit(ina226_t dev);

/* I2C address the device was found at (useful for logging). */
uint8_t ina226_address(ina226_t dev);

/* Bus voltage in millivolts (1.25 mV/LSB). */
esp_err_t ina226_bus_mv(ina226_t dev, int32_t *mv);

/* Current in milliamps. Positive = into the battery, negative = out of it. */
esp_err_t ina226_current_ma(ina226_t dev, float *ma);

/* Log every relevant register raw (CONFIG/CAL/SHUNT/BUS/CURRENT/POWER).
 * Diagnostics only — makes a frozen or misconfigured conversion obvious. */
void ina226_dump(ina226_t dev);
