/*
 * ina226.c — TI INA226 current/power monitor driver (minimal, polling only).
 *
 * Register map (datasheet SBOS547):
 *   0x00 CONFIG   0x01 SHUNT_V  0x02 BUS_V   0x03 POWER
 *   0x04 CURRENT  0x05 CAL      0xFE MFG_ID  0xFF DIE_ID
 */
#include "ina226.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "INA226";

#define REG_CONFIG  0x00
#define REG_BUS_V   0x02
#define REG_CURRENT 0x04
#define REG_CAL     0x05
#define REG_MFG_ID  0xFE
#define REG_DIE_ID  0xFF

#define MFG_ID_TI   0x5449 /* "TI" */
#define DIE_ID_226  0x2260

/* Full-scale shunt range is +-81.92 mV. */
#define SHUNT_FULL_SCALE_V 0.08192f
/* 1 LSB of the current register == Current_LSB. Pick 100 uA/LSB, which keeps
 * +-3.2 A in range — well above the IP2326's 1 A charge limit and the Tab5's
 * peak draw. */
#define CURRENT_LSB_A 0.0001f

/* CONFIG: AVG=16, VBUSCT=1.1 ms, VSHCT=1.1 ms, MODE=111 (shunt+bus, continuous).
 *
 * Bit layout (datasheet SBOS547, table 7):
 *   [15:13] AVG   [12:9] VBUSCT   [8:6] VSHCT   [5:3] MODE   [2:0] reserved
 *   AVG codes:    000=1 001=4 010=16 011=64 100=128 101=256 110=512 111=1024
 *   VBUSCT/VSHCT: 000=140us ... 100=1.1ms ... 111=8.244ms
 *   MODE codes:   000/100=power-down 001=shunt once 010=bus once
 *                 011=shunt+bus once 101=shunt cont 110=bus cont 111=both cont
 *
 * 010 0100 100 111 000  =>  0x5138.
 *
 * Getting MODE wrong is a nasty trap: 110 (bus-only) still updates the bus
 * voltage register, so the voltage looks perfectly healthy while the current
 * register silently freezes at a stale value — which then looks like a
 * charging fault rather than a configuration bug. */
#define CONFIG_VALUE 0x4938

struct ina226_state {
    i2c_master_dev_handle_t device;
    uint8_t address;
};

static esp_err_t rd16(ina226_t d, uint8_t reg, uint16_t *v)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(d->device, &reg, 1, buf, 2, -1);
    if (err != ESP_OK) return err;
    *v = (uint16_t)((buf[0] << 8) | buf[1]); /* big-endian */
    return ESP_OK;
}

static esp_err_t wr16(ina226_t d, uint8_t reg, uint16_t v)
{
    uint8_t buf[3] = {reg, (uint8_t)(v >> 8), (uint8_t)(v & 0xFF)};
    return i2c_master_transmit(d->device, buf, sizeof buf, -1);
}

/* Attach to `addr` and confirm it is really an INA226. */
static esp_err_t probe_addr(i2c_master_bus_handle_t bus, uint8_t addr, ina226_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };

    struct ina226_state *state = calloc(1, sizeof *state);
    if (!state) return ESP_ERR_NO_MEM;
    state->address = addr;

    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &state->device);
    if (err != ESP_OK) {
        free(state);
        return err;
    }

    uint16_t mfg = 0, die = 0;
    err = rd16(state, REG_MFG_ID, &mfg);
    if (err == ESP_OK) err = rd16(state, REG_DIE_ID, &die);

    if (err != ESP_OK || mfg != MFG_ID_TI || die != DIE_ID_226) {
        i2c_master_bus_rm_device(state->device);
        free(state);
        return ESP_ERR_NOT_FOUND;
    }

    *out = state;
    return ESP_OK;
}

esp_err_t ina226_init(i2c_master_bus_handle_t bus, ina226_t *out)
{
    if (!bus || !out) return ESP_ERR_INVALID_ARG;
    *out = NULL;

    /* The A0/A1 straps are not documented on the Tab5 schematic, so probe the
     * whole 0x40..0x4F window. The ID registers make a false positive
     * essentially impossible. */
    ina226_t dev = NULL;
    for (uint8_t a = 0x40; a <= 0x4F; a++) {
        if (probe_addr(bus, a, &dev) == ESP_OK) break;
        dev = NULL;
    }
    if (!dev) {
        ESP_LOGW(TAG, "no INA226 found on 0x40..0x4F");
        return ESP_ERR_NOT_FOUND;
    }

    /* Software reset first (CONFIG bit15 RST, self-clearing). Without this the
     * part can keep a stuck conversion state carried over from whatever ran
     * before us, in which case CONFIG/CAL read back correctly but CURRENT and
     * POWER never update — which looks exactly like a dead charger. */
    wr16(dev, REG_CONFIG, 0x8000);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Calibration: CAL = 0.00512 / (Current_LSB * Rshunt) */
    uint16_t cal = (uint16_t)(0.00512f / (CURRENT_LSB_A * INA226_SHUNT_OHMS) + 0.5f);
    esp_err_t err = wr16(dev, REG_CAL, cal);
    if (err == ESP_OK) err = wr16(dev, REG_CONFIG, CONFIG_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(dev->device);
        free(dev);
        return err;
    }

    /* Read the two writable registers back. If CONFIG does not come back as
     * CONFIG_VALUE the writes are not landing (wrong device, bus fault, or a
     * different variant) and every later reading would be meaningless. */
    uint16_t cfg_rd = 0, cal_rd = 0;
    rd16(dev, REG_CONFIG, &cfg_rd);
    rd16(dev, REG_CAL, &cal_rd);

    ESP_LOGI(TAG, "found at 0x%02X (cal=%u wrote/read %u, cfg wrote 0x%04X read 0x%04X%s)",
             dev->address, cal, cal_rd, CONFIG_VALUE, cfg_rd,
             cfg_rd == CONFIG_VALUE ? "" : " MISMATCH");
    *out = dev;
    return ESP_OK;
}

esp_err_t ina226_deinit(ina226_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_bus_rm_device(dev->device);
    free(dev);
    return err;
}

uint8_t ina226_address(ina226_t dev) { return dev ? dev->address : 0; }

esp_err_t ina226_bus_mv(ina226_t dev, int32_t *mv)
{
    if (!dev || !mv) return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    esp_err_t err = rd16(dev, REG_BUS_V, &raw);
    if (err != ESP_OK) return err;
    *mv = (int32_t)((float)raw * 1.25f); /* 1.25 mV/LSB */
    return ESP_OK;
}

esp_err_t ina226_current_ma(ina226_t dev, float *ma)
{
    if (!dev || !ma) return ESP_ERR_INVALID_ARG;
    uint16_t raw;
    esp_err_t err = rd16(dev, REG_CURRENT, &raw);
    if (err != ESP_OK) return err;
    *ma = (float)(int16_t)raw * CURRENT_LSB_A * 1000.0f; /* signed */
    return ESP_OK;
}

/* Dump every register that matters, raw. Used by the boot/heartbeat diagnostics
 * so a frozen or misconfigured conversion is visible as data rather than
 * guessed at from a suspiciously stable milliamp reading. */
void ina226_dump(ina226_t dev)
{
    if (!dev) {
        ESP_LOGW(TAG, "dump: no device");
        return;
    }
    uint16_t cfg = 0, cal = 0, sh = 0, bus = 0, cur = 0, pwr = 0;
    rd16(dev, REG_CONFIG, &cfg);
    rd16(dev, REG_CAL, &cal);
    rd16(dev, 0x01, &sh);
    rd16(dev, REG_BUS_V, &bus);
    rd16(dev, REG_CURRENT, &cur);
    rd16(dev, 0x03, &pwr);
    ESP_LOGI(TAG, "0x%02X cfg=0x%04X(exp 0x%04X mode=%u) cal=%u shunt=0x%04X(%dmV) "
                  "bus=0x%04X(%dmV) cur=0x%04X(%+d) pwr=0x%04X",
             dev->address, cfg, CONFIG_VALUE, (unsigned)((cfg >> 3) & 0x7), cal,
             sh, (int)((int16_t)sh * 2), bus, (int)(bus * 1.25f),
             cur, (int)(int16_t)cur, pwr);
}
