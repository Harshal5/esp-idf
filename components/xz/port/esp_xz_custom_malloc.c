#include <stdlib.h>
#include <esp_log.h>
#include <soc/soc.h>

extern uint32_t _heap_start, _heap_end;

static uint8_t* heap_pool = (uint8_t*) &_heap_start;
static uint32_t heap_used_offset = 0;

void* esp_xz_custom_malloc(size_t size)
{
    void* p = NULL;

    if (heap_used_offset + size < _heap_end) {
        p = &heap_pool[heap_used_offset];
        heap_used_offset += size;
    }
    // ESP_EARLY_LOGI("esp_xz_custom_malloc", "heap_used_offset=%ld size=%u", heap_used_offset, size);
    return p;
}

void esp_xz_custom_free(void *ptr)
{

}
