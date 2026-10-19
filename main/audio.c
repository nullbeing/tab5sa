/*
 * audio.c — ES8388 speaker output via esp_codec_dev.
 *
 * audio_play() (called from the demod task) interleaves mono→stereo and pushes
 * into a StreamBuffer; audio_out_task drains it into esp_codec_dev_write(), which
 * blocks on the I2S DMA — keeping that blocking call off the real-time demod path.
 */
#include "audio.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"

#include "bsp_tab5.h"

static const char *TAG = "audio";

#define SB_BYTES (96 * 1024) /* ~0.5 s of 48k stereo int16 */

static esp_codec_dev_handle_t s_codec;
static StreamBufferHandle_t s_sb;

static void audio_out_task(void *arg)
{
    (void)arg;
    static uint8_t buf[4096];
    /* Zero-filled: used to keep the I2S TX fed when nothing is playing. */
    static const uint8_t silence[4096];

    while (1) {
        /* Bounded wait, not portMAX_DELAY: the producer (the demod task) stops
         * sending the moment the receiver is stopped or the dongle is absent.
         * With an infinite wait this task would then block forever, the I2S TX
         * DMA would underrun, and the codec would keep replaying its last
         * buffer — which the speaker renders as continuous noise. */
        size_t n = xStreamBufferReceive(s_sb, buf, sizeof buf, pdMS_TO_TICKS(20));
        if (n) {
            esp_codec_dev_write(s_codec, buf, n);
        } else {
            /* ~21 ms of silence at 48 kHz stereo 16-bit. Writing it also paces
             * this loop, since the codec write blocks on the DMA. */
            esp_codec_dev_write(s_codec, silence, sizeof silence);
        }
    }
}

void audio_init(void)
{
    s_codec = bsp_tab5_audio_speaker_init();
    if (!s_codec) {
        ESP_LOGE(TAG, "speaker codec init failed");
        return;
    }
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = AUDIO_SAMPLE_RATE,
    };
    if (esp_codec_dev_open(s_codec, &fs) != 0) {
        ESP_LOGE(TAG, "codec open failed");
        return;
    }
    esp_codec_dev_set_out_vol(s_codec, AUDIO_VOL_DEFAULT);

    s_sb = xStreamBufferCreate(SB_BYTES, 1);
    xTaskCreatePinnedToCore(audio_out_task, "audio_out", 4096, NULL, 9, NULL, 1);
    ESP_LOGI(TAG, "speaker up at %d Hz", AUDIO_SAMPLE_RATE);
}

void audio_play(const int16_t *mono, int n)
{
    if (!s_sb) return;
    static int16_t st[512 * 2];
    int off = 0;
    while (off < n) {
        int chunk = n - off;
        if (chunk > 512) chunk = 512;
        for (int i = 0; i < chunk; i++) {
            int16_t s = mono[off + i];
            st[2 * i] = s;
            st[2 * i + 1] = s;
        }
        /* timeout 0: drop on overflow rather than block the demod task */
        xStreamBufferSend(s_sb, st, (size_t)chunk * 2 * sizeof(int16_t), 0);
        off += chunk;
    }
}

void audio_set_volume(int vol)
{
    if (s_codec) esp_codec_dev_set_out_vol(s_codec, vol);
}
