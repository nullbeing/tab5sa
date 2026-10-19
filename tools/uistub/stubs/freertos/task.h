#pragma once
#include "FreeRTOS.h"

void vTaskDelay(int t);
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack,
                                   void *arg, unsigned prio, void *handle, int core);
