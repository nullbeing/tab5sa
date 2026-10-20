/*
 * Tab5-SDR — standalone RTL-SDR receiver on the M5Stack Tab5 (ESP32-P4).
 *
 * Stage 1: BSP bring-up + RGB565 test pattern (panel proof).
 * Stage 2: USB host power + librtlsdr enumeration (see sdr.c).
 * Stage 3: stream IQ into a PSRAM ring buffer (see sdr.c).
 * Stage 4: FFT spectrum + waterfall (see dsp.c).
 *
 * app_main handles boot/enumeration with solid-colour status, then hands the
 * screen to the DSP renderer and just emits a periodic serial heartbeat.
 */
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_app_desc.h"

#include "bsp_tab5.h"
#include "sdr.h"
#include "dsp.h"
#include "audio.h"
#include "ui.h"
#include "shot.h"

static const char *TAG = "tab5sa";

/* Tab5 panel: 720x1280, RGB565 (see bsp_tab5.c). */
#define PANEL_W 720
#define PANEL_H 1280

#define RGB565(r, g, b) \
    ((uint16_t)(((((r) >> 3) & 0x1F) << 11) | ((((g) >> 2) & 0x3F) << 5) | (((b) >> 3) & 0x1F)))

static uint16_t *fb(void) { return (uint16_t *)bsp_tab5_display_get_frame_buffer(0); }

static void draw_test_pattern(void)
{
    uint16_t *f = fb();
    if (!f) {
        ESP_LOGE(TAG, "no framebuffer");
        return;
    }
    static const uint16_t bars[8] = {
        RGB565(0, 0, 0),     RGB565(0, 0, 255),   RGB565(0, 255, 0),   RGB565(0, 255, 255),
        RGB565(255, 0, 0),   RGB565(255, 0, 255), RGB565(255, 255, 0), RGB565(255, 255, 255),
    };
    for (int y = 0; y < PANEL_H; y++) {
        uint16_t *row = f + (size_t)y * PANEL_W;
        for (int x = 0; x < PANEL_W; x++) row[x] = bars[(x * 8) / PANEL_W];
    }
    bsp_tab5_display_flush(0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Tab5SA %s boot — free heap %" PRIu32 ", PSRAM %u",
             esp_app_get_description()->version,
             esp_get_free_heap_size(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ESP_ERROR_CHECK(bsp_tab5_init(&(bsp_tab5_config_t){
        .display.fb_num = 2, /* double buffer: PPA writes one while the other scans out */
        .touch.interrupt = true,
        .wifi.enable = false,
        .bluetooth.enable = false,
    }));
    bsp_tab5_display_set_brightness(80);
    draw_test_pattern();

    /* USB-A 5V VBUS is off by default — power the dongle, then enumerate.
     * The dongle shares this port with the flashing/serial link, so it is
     * normally ABSENT. sdr_start() is non-blocking: it brings up the USB host
     * and a supervisor task that keeps looking for a dongle in the background,
     * and streams whenever one appears. The UI comes up either way. */
    bsp_tab5_set_usb_host_power(true);
    vTaskDelay(pdMS_TO_TICKS(300));

    sdr_start(SDR_DEFAULT_CENTER_FREQ, SDR_DEFAULT_SAMPLE_RATE);

    /* Speaker up, then demod + spectrum console take over the screen. */
    audio_init();
    dsp_start();
    ui_start();
    /* After ui_start(): the capture path reads the panel framebuffer ui.c owns. */
    shot_start();

    /* Heartbeat: produced= is what proves IQ is actually arriving (dropped=
     * stays 0 both when the consumer keeps up AND when nothing is produced).
     * The battery line proves the INA226 answered and charging is enabled. */
    int hb = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        sdr_stats_t st;
        sdr_get_stats(&st);
        hb++;

        bsp_tab5_battery_t bat;
        bsp_tab5_battery_read(&bat);
        /* Orientation is appended to the heartbeat rather than logged once at
         * boot: the USB-serial port is shared with flashing, so early boot lines
         * are easy to miss. Raw mg values are printed so a wrong axis choice can
         * be diagnosed from a single capture. */
        bsp_tab5_orient_t ori;
        bsp_tab5_orientation_get(&ori);
        const char *ori_s = !ori.present ? "no IMU"
                          : ori.flipped ? "FLIP" : "norm";
        /* Raw counts and the live range are printed next to the converted mg
         * values so the two can be cross-checked: at rest the magnitude must
         * be ~1000 mg, which is what confirms the range register and the
         * assumed sensitivity agree. Same numbers are needed to work out which
         * axis is "up" on this board. */
        if (bat.present) {
            ESP_LOGI(TAG, "%s, %.3f MHz, produced=%llu dropped=%llu | bat %d.%02dV %+.0fmA %d%% %s%s | imu %s r%u %+d/%+d/%+d mg (raw %+d/%+d/%+d)",
                     st.present ? st.tuner : "no dongle",
                     st.center_freq / 1e6, (unsigned long long)st.produced_bytes,
                     (unsigned long long)st.dropped_bytes,
                     (int)(bat.mv / 1000), (int)((bat.mv % 1000) / 10),
                     bat.ma, bat.percent,
                     bat.charging ? "CHG" : "dischg",
                     bsp_tab5_charge_enabled() ? "" : " (CHG_EN off!)",
                     ori_s, ori.range, ori.ax, ori.ay, ori.az,
                     ori.rx, ori.ry, ori.rz);
        } else {
            ESP_LOGI(TAG, "%s, %.3f MHz, produced=%llu dropped=%llu | no INA226, chg_en=%d | imu %s r%u %+d/%+d/%+d mg (raw %+d/%+d/%+d)",
                     st.present ? st.tuner : "no dongle",
                     st.center_freq / 1e6, (unsigned long long)st.produced_bytes,
                     (unsigned long long)st.dropped_bytes,
                     (int)bsp_tab5_charge_enabled(),
                     ori_s, ori.range, ori.ax, ori.ay, ori.az,
                     ori.rx, ori.ry, ori.rz);
        }
        /* Raw register dump periodically: enough to diagnose a stuck
         * conversion or a charge-enable that did not land, without flooding
         * the console. */
        if (hb % 6 == 0) bsp_tab5_battery_dump();
    }
}
