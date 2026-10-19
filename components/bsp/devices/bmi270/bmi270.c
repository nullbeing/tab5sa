/*
 * bmi270.c — Bosch BMI270 accelerometer (polling only, no FIFO, no interrupts).
 *
 * Only what is needed for "is the device upside down": the accelerometer gives
 * gravity, which is an absolute reference. The gyro is not used — integrating
 * angular rate to get orientation drifts, and gravity is already available.
 *
 * The BMI270 will not produce data until Bosch's ~8 KiB configuration blob has
 * been uploaded, so init is: soft reset -> disable power save -> upload blob ->
 * wait for init_ok -> enable accelerometer.
 */
#include "bmi270.h"
#include "bmi270_config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BMI270";

/* Register map (BMI270 datasheet rev 1.6) */
#define REG_CHIP_ID        0x00
#define REG_ACC_X_LSB      0x0C /* 6 bytes: X,Y,Z little-endian int16 */
#define REG_INTERNAL_STATUS 0x21
#define REG_ACC_CONF       0x40
#define REG_ACC_RANGE      0x41
#define REG_INIT_CTRL      0x59
#define REG_INIT_ADDR_0    0x5B
#define REG_INIT_ADDR_1    0x5C
#define REG_INIT_DATA      0x5E
#define REG_PWR_CONF       0x7C
#define REG_PWR_CTRL       0x7D
#define REG_CMD            0x7E

#define CHIP_ID_BMI270     0x24
#define CMD_SOFTRESET      0xB6

/* Request +-4 g: enough headroom to never clip when the panel is tapped or
 * flipped quickly, while keeping ~0.12 mg of resolution. The range actually in
 * effect is read back after writing and used for scaling, so a range write that
 * does not stick cannot silently scale every reading by 2x. */
#define ACC_RANGE_4G       0x01

/* LSB per g for each ACC_RANGE setting: 0=+-2g, 1=+-4g, 2=+-8g, 3=+-16g. */
static uint16_t range_lsb_per_g(uint8_t range)
{
    switch (range & 0x03) {
    case 0:  return 16384;
    case 1:  return 8192;
    case 2:  return 4096;
    default: return 2048;
    }
}

/* Integer square root (Newton). Avoids pulling in libm for one sqrt. */
static int32_t isqrt32(int32_t v)
{
    if (v <= 0) return 0;
    int32_t x = v, y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + v / x) / 2;
    }
    return x;
}

/*
 * Derive counts-per-g from gravity instead of trusting the range register.
 *
 * At rest the acceleration magnitude is exactly 1 g, so the mean magnitude of a
 * few samples IS the sensitivity in counts per g. On this board ACC_RANGE reads
 * back as +-4 g (8192 LSB/g) while the data is actually scaled like +-8 g
 * (4096 LSB/g), which halves every reported value. Measured gravity removes
 * that ambiguity; snapping to the nearest standard range keeps the result exact
 * and tolerant of a device that is not perfectly still during boot.
 */
static uint16_t calibrate_lsb_per_g(bmi270_t dev, uint16_t fallback)
{
    static const uint16_t std_lsb[] = { 2048, 4096, 8192, 16384 };
    int64_t sum = 0;
    int n = 0;

    for (int i = 0; i < 16; i++) {
        int16_t rx, ry, rz;
        if (bmi270_accel_raw(dev, &rx, &ry, &rz) == ESP_OK) {
            int32_t m2 = (int32_t)rx * rx + (int32_t)ry * ry + (int32_t)rz * rz;
            sum += isqrt32(m2);
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n == 0) return fallback;

    int32_t mean = (int32_t)(sum / n);

    uint16_t best = fallback;
    int32_t best_d = -1;
    for (unsigned i = 0; i < sizeof std_lsb / sizeof std_lsb[0]; i++) {
        int32_t d = mean > (int32_t)std_lsb[i] ? mean - (int32_t)std_lsb[i]
                                              : (int32_t)std_lsb[i] - mean;
        if (best_d < 0 || d < best_d) {
            best_d = d;
            best = std_lsb[i];
        }
    }

    /* Reject a nonsense measurement (moving at boot, or no real data) and keep
     * the register-derived value rather than scaling by a bogus factor. */
    if (mean < 256 || mean > 32000) {
        ESP_LOGW(TAG, "gravity %ld counts is implausible, keeping %u LSB/g",
                 (long)mean, fallback);
        return fallback;
    }

    ESP_LOGI(TAG, "gravity %ld counts -> %u LSB/g (range register implies %u)",
             (long)mean, best, fallback);
    return best;
}

/*
 * Run at 100 kHz, not 400 kHz.
 *
 * On the Tab5 the BMI270 sits behind the Q13A/Q13B bidirectional level
 * translators, whose rise time with the 4.7k pull-ups is ~500 ns. That busts the
 * 300 ns limit of I2C fast mode, and the symptom is nasty: every write ACKs
 * normally, but bytes arrive corrupted, so the 8 KiB configuration program the
 * sensor executes is garbage and INIT_OK never asserts. At 100 kHz the budget is
 * 1000 ns and it loads reliably. The cost is a one-off ~0.8 s during boot and
 * sub-millisecond polls afterwards. */
#define BMI270_I2C_HZ      100000

/* ACC_CONF = odr[7:4] | bwp[3:0]. 100 Hz, normal filtering. */
#define ACC_CONF_100HZ_NORM 0x82

struct bmi270_state {
    i2c_master_dev_handle_t device;
    uint8_t address;
    uint16_t lsb_per_g; /* derived from the ACC_RANGE readback at init */
    uint8_t range;      /* ACC_RANGE value last read back             */
};

static esp_err_t rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(d, &reg, 1, buf, n, -1);
}

static esp_err_t wr(i2c_master_dev_handle_t d, uint8_t reg, const uint8_t *buf, size_t n)
{
    uint8_t tmp[64];
    if (n + 1 > sizeof tmp) return ESP_ERR_INVALID_SIZE;
    tmp[0] = reg;
    memcpy(tmp + 1, buf, n);
    return i2c_master_transmit(d, tmp, n + 1, -1);
}

static esp_err_t wr8(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v)
{
    return wr(d, reg, &v, 1);
}

/* Attach to `addr` and confirm the chip ID. */
static esp_err_t probe_addr(i2c_master_bus_handle_t bus, uint8_t addr, bmi270_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = BMI270_I2C_HZ,
    };

    struct bmi270_state *st = calloc(1, sizeof *st);
    if (!st) return ESP_ERR_NO_MEM;
    st->address = addr;

    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &st->device);
    if (err != ESP_OK) {
        free(st);
        return err;
    }

    uint8_t id = 0;
    err = rd(st->device, REG_CHIP_ID, &id, 1);
    if (err != ESP_OK || id != CHIP_ID_BMI270) {
        i2c_master_bus_rm_device(st->device);
        free(st);
        return ESP_ERR_NOT_FOUND;
    }

    *out = st;
    return ESP_OK;
}

/* Read back the handful of registers that explain an init failure. INIT_DATA
 * itself is write-only, so this cannot verify the payload — but it does show
 * whether the control writes landed and what the sensor's internal MCU thinks. */
static void dump_regs(bmi270_t dev, const char *when)
{
    uint8_t id = 0, pwr_conf = 0, pwr_ctrl = 0, init_ctrl = 0;
    uint8_t addr0 = 0, addr1 = 0, int_st = 0, acc_conf = 0, acc_range = 0;
    rd(dev->device, REG_CHIP_ID, &id, 1);
    rd(dev->device, REG_PWR_CONF, &pwr_conf, 1);
    rd(dev->device, REG_PWR_CTRL, &pwr_ctrl, 1);
    rd(dev->device, REG_INIT_CTRL, &init_ctrl, 1);
    rd(dev->device, REG_INIT_ADDR_0, &addr0, 1);
    rd(dev->device, REG_INIT_ADDR_1, &addr1, 1);
    rd(dev->device, REG_INTERNAL_STATUS, &int_st, 1);
    rd(dev->device, REG_ACC_CONF, &acc_conf, 1);
    rd(dev->device, REG_ACC_RANGE, &acc_range, 1);
    ESP_LOGI(TAG, "%s: id=0x%02X pwr_conf=0x%02X pwr_ctrl=0x%02X init_ctrl=0x%02X "
                  "addr=%X:%X int_st=0x%02X acc_conf=0x%02X acc_range=0x%02X",
             when, id, pwr_conf, pwr_ctrl, init_ctrl, addr1, addr0, int_st,
             acc_conf, acc_range);
}

/* Upload the configuration blob. Returns ESP_OK once the chip reports init_ok.
 *
 * `burst` is the number of payload bytes per INIT_DATA write. INIT_DATA
 * auto-increments its address, so 32 bytes may be sent as one transfer or split
 * into smaller ones with identical meaning — which makes it a clean test for
 * long-transfer corruption.
 */
static esp_err_t upload_config(bmi270_t dev, int chunk_delay_ms, int burst)
{
    const int chunk = 32;
    const int pages = bmi270_config_size / chunk;

    esp_err_t err = wr8(dev->device, REG_INIT_CTRL, 0x00);
    if (err != ESP_OK) return err;

    for (int i = 0; i < pages; i++) {
        /* INIT_ADDR is a 12-bit address in 16-bit WORDS, not a page number:
         * the hardware takes the byte offset, divides by two, and splits that
         * into a low nibble (INIT_ADDR_0) and the remaining high bits
         * (INIT_ADDR_1). Bosch's upload_file() does exactly:
         *     addr[0] = (index / 2) & 0x0F
         *     addr[1] = (index / 2) >> 4
         * with `index` the byte offset. For a 32-byte chunk this means
         * addr[0] is always 0 and addr[1] counts 0..255.
         *
         * Using the chunk number directly instead (addr[0] = i & 0x0F) sends
         * every 16th block to the wrong place: all control writes still ACK,
         * but the config program is scattered, so the sensor answers INIT_ERR
         * and never produces data. */
        uint16_t word = (uint16_t)((i * chunk) / 2);
        uint8_t addr[2] = { (uint8_t)(word & 0x0F), (uint8_t)(word >> 4) };
        err = wr(dev->device, REG_INIT_ADDR_0, addr, sizeof addr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "addr write failed at page %d: %s", i, esp_err_to_name(err));
            return err;
        }

        for (int off = 0; off < chunk; off += burst) {
            err = wr(dev->device, REG_INIT_DATA, &bmi270_config[i * chunk + off], burst);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "data write failed at page %d: %s", i, esp_err_to_name(err));
                return err;
            }
        }
        if (chunk_delay_ms) vTaskDelay(pdMS_TO_TICKS(chunk_delay_ms));
    }

    err = wr8(dev->device, REG_INIT_CTRL, 0x01);
    if (err != ESP_OK) return err;

    /* The blob is a program the sensor executes; init_ok (INTERNAL_STATUS bit0)
     * tells us it actually ran. Without this check a silent upload failure would
     * surface later as "all-zero accelerometer data". */
    for (int i = 0; i < 40; i++) { /* up to ~200 ms; the MCU finishes in ~20 */
        uint8_t st = 0;
        if (rd(dev->device, REG_INTERNAL_STATUS, &st, 1) == ESP_OK && (st & 0x01))
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /* bits[3:1] carry the reason the internal MCU gave up, which is the only
     * clue available when the upload is silently corrupt. */
    uint8_t st = 0;
    rd(dev->device, REG_INTERNAL_STATUS, &st, 1);
    ESP_LOGW(TAG, "upload rejected: INTERNAL_STATUS=0x%02X message=%d",
             st, (st & 0x0E) >> 1);
    return ESP_ERR_TIMEOUT;
}

esp_err_t bmi270_init(i2c_master_bus_handle_t bus, bmi270_t *out)
{
    if (!bus || !out) return ESP_ERR_INVALID_ARG;
    *out = NULL;

    /* SDO strap is not documented for the Tab5, so try both addresses. */
    bmi270_t dev = NULL;
    if (probe_addr(bus, BMI270_ADDR_LOW, &dev) != ESP_OK) dev = NULL;
    if (!dev && probe_addr(bus, BMI270_ADDR_HIGH, &dev) != ESP_OK) dev = NULL;
    if (!dev) {
        ESP_LOGW(TAG, "no BMI270 on 0x%02X/0x%02X", BMI270_ADDR_LOW, BMI270_ADDR_HIGH);
        return ESP_ERR_NOT_FOUND;
    }

    /* The upload runs a program on the sensor's internal MCU. The subtle part
     * is the address encoding (see upload_config). A few retries matter here:
     * the 8 KiB upload is the one long transaction on this level-shifted bus and
     * fails intermittently at boot, so a single attempt is not reliable. */
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        err = wr8(dev->device, REG_CMD, CMD_SOFTRESET);
        if (err != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20)); /* datasheet: >450 us */

        /* Advanced power save must be off before the blob can be uploaded. */
        err = wr8(dev->device, REG_PWR_CONF, 0x00);
        if (err != ESP_OK) break;

        err = upload_config(dev, 0, 32);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "config upload attempt %d failed: %s", attempt,
                 esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config upload failed");
        dump_regs(dev, "after failure");
        goto fail;
    }

    /* The accelerometer is still off here (a freshly uploaded config leaves
     * PWR_CTRL at 0), which is the only state in which ACC_CONF may change. */
    err = wr8(dev->device, REG_ACC_RANGE, ACC_RANGE_4G);
    if (err == ESP_OK) err = wr8(dev->device, REG_ACC_CONF, ACC_CONF_100HZ_NORM);
    if (err != ESP_OK) goto fail;

    /* Trust the readback, not the request. */
    uint8_t range = ACC_RANGE_4G;
    if (rd(dev->device, REG_ACC_RANGE, &range, 1) != ESP_OK) range = ACC_RANGE_4G;
    dev->range = range;
    dev->lsb_per_g = range_lsb_per_g(range);

    /* PWR_CTRL bit2 = acc_en. */
    err = wr8(dev->device, REG_PWR_CTRL, 0x04);
    if (err != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(50)); /* first sample */

    /* Prove data actually moves before declaring victory: a config that loaded
     * but never produces samples is just as useless as no config at all. */
    int16_t x = 0, y = 0, z = 0;
    if (bmi270_accel_raw(dev, &x, &y, &z) != ESP_OK || (x == 0 && y == 0 && z == 0)) {
        ESP_LOGE(TAG, "initialised but first sample is 0/0/0");
        err = ESP_ERR_INVALID_STATE;
        goto fail;
    }

    /* Now that samples flow, work out the real sensitivity from gravity. */
    dev->lsb_per_g = calibrate_lsb_per_g(dev, range_lsb_per_g(range));

    int16_t mx, my, mz;
    bmi270_accel_mg(dev, &mx, &my, &mz);
    ESP_LOGI(TAG, "found at 0x%02X (range=0x%02X, %u LSB/g), gravity %+d/%+d/%+d mg",
             dev->address, range, (unsigned)dev->lsb_per_g, mx, my, mz);
    *out = dev;
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(dev->device);
    free(dev);
    return err;
}

esp_err_t bmi270_deinit(bmi270_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_bus_rm_device(dev->device);
    free(dev);
    return err;
}

uint8_t bmi270_address(bmi270_t dev) { return dev ? dev->address : 0; }

esp_err_t bmi270_accel_raw(bmi270_t dev, int16_t *x, int16_t *y, int16_t *z)
{
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;

    uint8_t b[6];
    esp_err_t err = rd(dev->device, REG_ACC_X_LSB, b, sizeof b);
    if (err != ESP_OK) return err;

    /* Little-endian int16 per axis. */
    *x = (int16_t)((b[1] << 8) | b[0]);
    *y = (int16_t)((b[3] << 8) | b[2]);
    *z = (int16_t)((b[5] << 8) | b[4]);
    return ESP_OK;
}

uint8_t bmi270_accel_range(bmi270_t dev)
{
    return dev ? dev->range : 0;
}

esp_err_t bmi270_accel_mg(bmi270_t dev, int16_t *x, int16_t *y, int16_t *z)
{
    if (!dev || !x || !y || !z) return ESP_ERR_INVALID_ARG;

    int16_t rx, ry, rz;
    esp_err_t err = bmi270_accel_raw(dev, &rx, &ry, &rz);
    if (err != ESP_OK) return err;

    const int32_t scale = dev->lsb_per_g ? dev->lsb_per_g : 8192;
    *x = (int16_t)((int32_t)rx * 1000 / scale);
    *y = (int16_t)((int32_t)ry * 1000 / scale);
    *z = (int16_t)((int32_t)rz * 1000 / scale);
    return ESP_OK;
}
