#pragma once
#include <stdio.h>

#define ESP_LOGI(t, ...) do { printf("[I %s] ", t); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGW(t, ...) do { printf("[W %s] ", t); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_LOGE(t, ...) do { printf("[E %s] ", t); printf(__VA_ARGS__); printf("\n"); } while (0)
#define ESP_ERROR_CHECK(x) do { (void)(x); } while (0)
