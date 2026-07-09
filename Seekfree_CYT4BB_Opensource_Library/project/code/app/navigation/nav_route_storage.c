#include "nav_route_storage.h"

#include <stddef.h>
#include <string.h>

#include "zf_driver_flash.h"

#define NAV_ROUTE_STORAGE_MAGIC    0x4E415652u
#define NAV_ROUTE_STORAGE_VERSION  1u
#define NAV_ROUTE_STORAGE_PAGE     (FLASH_PAGE_NUM - 1u)
#define NAV_ROUTE_STORAGE_SECTOR   0u

typedef struct {
    uint32 magic;
    uint32 version;
    uint32 route_len;
    uint32 checksum;
    Nav_Segment_t route[NAV_RECORD_MAX_SEGMENTS];
} Nav_Route_Storage_Image_t;

#define NAV_ROUTE_STORAGE_WORDS \
    ((uint32)((sizeof(Nav_Route_Storage_Image_t) + sizeof(uint32) - 1u) / sizeof(uint32)))

typedef union {
    Nav_Route_Storage_Image_t image;
    uint32 words[NAV_ROUTE_STORAGE_WORDS];
} Nav_Route_Storage_Buffer_t;

static Nav_Route_Storage_Buffer_t g_storage_buffer;
static bool g_storage_ready;

static void storage_ensure_init(void)
{
    if (!g_storage_ready) {
        flash_init();
        g_storage_ready = true;
    }
}

static uint32 storage_checksum(const uint32 *words, uint32 word_count)
{
    uint32 checksum = 2166136261u;

    for (uint32 i = 0u; i < word_count; i++) {
        checksum ^= words[i];
        checksum *= 16777619u;
    }

    return checksum;
}

void nav_route_storage_init(void)
{
    storage_ensure_init();
}

bool nav_route_storage_save(const Nav_Segment_t *route, uint8 route_len)
{
    if (route == NULL ||
        route_len == 0u ||
        route_len > NAV_RECORD_MAX_SEGMENTS ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    memset(&g_storage_buffer, 0xFF, sizeof(g_storage_buffer));
    g_storage_buffer.image.magic = NAV_ROUTE_STORAGE_MAGIC;
    g_storage_buffer.image.version = NAV_ROUTE_STORAGE_VERSION;
    g_storage_buffer.image.route_len = route_len;
    g_storage_buffer.image.checksum = 0u;

    for (uint8 i = 0u; i < route_len; i++) {
        g_storage_buffer.image.route[i] = route[i];
    }

    g_storage_buffer.image.checksum =
        storage_checksum(g_storage_buffer.words, NAV_ROUTE_STORAGE_WORDS);

    flash_write_page(NAV_ROUTE_STORAGE_SECTOR,
                     NAV_ROUTE_STORAGE_PAGE,
                     g_storage_buffer.words,
                     NAV_ROUTE_STORAGE_WORDS);

    return true;
}

bool nav_route_storage_load(Nav_Segment_t *route,
                            uint8 max_route_len,
                            uint8 *route_len)
{
    uint32 stored_checksum;
    uint8 stored_route_len;

    if (route == NULL ||
        route_len == NULL ||
        max_route_len == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    flash_read_page(NAV_ROUTE_STORAGE_SECTOR,
                    NAV_ROUTE_STORAGE_PAGE,
                    g_storage_buffer.words,
                    NAV_ROUTE_STORAGE_WORDS);

    if (g_storage_buffer.image.magic != NAV_ROUTE_STORAGE_MAGIC ||
        g_storage_buffer.image.version != NAV_ROUTE_STORAGE_VERSION ||
        g_storage_buffer.image.route_len == 0u ||
        g_storage_buffer.image.route_len > NAV_RECORD_MAX_SEGMENTS ||
        g_storage_buffer.image.route_len > max_route_len) {
        return false;
    }

    stored_route_len = (uint8)g_storage_buffer.image.route_len;
    stored_checksum = g_storage_buffer.image.checksum;
    g_storage_buffer.image.checksum = 0u;

    if (stored_checksum !=
        storage_checksum(g_storage_buffer.words, NAV_ROUTE_STORAGE_WORDS)) {
        g_storage_buffer.image.checksum = stored_checksum;
        return false;
    }

    g_storage_buffer.image.checksum = stored_checksum;

    for (uint8 i = 0u; i < stored_route_len; i++) {
        route[i] = g_storage_buffer.image.route[i];
    }

    *route_len = stored_route_len;
    return true;
}
