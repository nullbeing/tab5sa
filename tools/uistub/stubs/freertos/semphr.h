/*
 * semphr.h stub for the host preview.
 *
 * The preview runs ui.c on a single thread, so the panel mutex that guards
 * present() against ui_panel_snapshot() has nothing to contend with. These are
 * no-ops with the right signatures; the real firmware uses the FreeRTOS ones.
 */
#pragma once
#include <stdint.h>

#ifndef portMAX_DELAY
#define portMAX_DELAY 0xFFFFFFFFu
#endif

typedef void *SemaphoreHandle_t;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    static int dummy;
    return (SemaphoreHandle_t)&dummy;
}

static inline int xSemaphoreTake(SemaphoreHandle_t s, uint32_t ticks)
{
    (void)s;
    (void)ticks;
    return 1;
}

static inline int xSemaphoreGive(SemaphoreHandle_t s)
{
    (void)s;
    return 1;
}
