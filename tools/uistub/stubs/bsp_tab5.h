#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * Host-side stand-in for components/bsp/inc/bsp_tab5.h.
 *
 * Only the pieces ui.c actually touches are declared. Keeping this file small is
 * the point: if ui.c starts calling a new BSP function, the preview build breaks
 * here instead of silently drifting from the firmware.
 */

/* Display: the stub PPA writes into a fake panel framebuffer. */
void *bsp_tab5_display_get_frame_buffer(int fb_index);
void bsp_tab5_display_flush(int fb_index);

/* Touch: same struct shape as the esp_lcd_touch driver. */
typedef struct {
    int x, y;
} esp_lcd_touch_point_data_t;

int bsp_tab5_touch_read(esp_lcd_touch_point_data_t *points, uint8_t max_points);

/* Battery (INA226). */
typedef struct {
    bool present;
    int32_t mv;
    float ma;
    int percent;
    bool charging;
    bool external;
} bsp_tab5_battery_t;

void bsp_tab5_battery_read(bsp_tab5_battery_t *out);

/* Orientation (BMI270). */
typedef struct {
    bool present;
    bool flipped;
    int16_t ax, ay, az;
    int16_t rx, ry, rz;
    uint8_t range;
} bsp_tab5_orient_t;

bool bsp_tab5_orientation_poll(void);
bool bsp_tab5_orientation_set_upright(void);
void bsp_tab5_orientation_get(bsp_tab5_orient_t *out);
