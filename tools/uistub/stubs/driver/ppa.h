#pragma once
#include <stdint.h>

/*
 * Minimal stand-in for the ESP32-P4 PPA (Pixel Processing Accelerator).
 *
 * The stub actually performs the scale+rotate, so the preview image is what the
 * panel would show — including the 180° flip — rather than a raw dump of the
 * landscape buffer.
 */
typedef void *ppa_client_handle_t;
typedef enum { PPA_OPERATION_SRM } ppa_operation_t;
typedef enum { PPA_SRM_COLOR_MODE_RGB565 } ppa_srm_color_mode_t;

typedef enum {
    PPA_SRM_ROTATION_ANGLE_0 = 0,
    PPA_SRM_ROTATION_ANGLE_90 = 90,
    PPA_SRM_ROTATION_ANGLE_180 = 180,
    PPA_SRM_ROTATION_ANGLE_270 = 270,
} ppa_srm_rotation_angle_t;

typedef struct {
    void *buffer;
    uint32_t pic_w, pic_h, block_w, block_h;
    ppa_srm_color_mode_t srm_cm;
} ppa_in_t;

typedef struct {
    void *buffer;
    uint32_t buffer_size, pic_w, pic_h;
    ppa_srm_color_mode_t srm_cm;
} ppa_out_t;

typedef struct {
    ppa_in_t in;
    ppa_out_t out;
    ppa_srm_rotation_angle_t rotation_angle;
    float scale_x, scale_y;
} ppa_srm_oper_config_t;

typedef struct { ppa_operation_t oper_type; } ppa_client_config_t;

static inline int ppa_register_client(ppa_client_config_t *c, ppa_client_handle_t *h)
{
    (void)c;
    *h = (void *)1;
    return 0;
}

void ppa_do_scale_rotate_mirror(ppa_client_handle_t h, ppa_srm_oper_config_t *c);
