#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void* esp_xz_custom_malloc(size_t size);
void esp_xz_custom_free(void *ptr);

#ifdef __cplusplus
}
#endif
