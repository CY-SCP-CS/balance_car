#include "nav_route_storage.h"

#include <stddef.h>
#include <string.h>

#include "zf_driver_flash.h"

#define NAV_ROUTE_STORAGE_MAGIC    0x4E415652u
#define NAV_ROUTE_STORAGE_VERSION  2u
#define NAV_ROUTE_STORAGE_SLOT_COUNT  3u
#define NAV_ROUTE_STORAGE_FIRST_PAGE  (FLASH_PAGE_NUM - NAV_ROUTE_STORAGE_SLOT_COUNT)
#define NAV_ROUTE_STORAGE_SECTOR   0u

typedef struct {
    uint32 magic;
    uint32 version;
    uint32 sequence;
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

typedef char Nav_Route_Storage_Page_Fit_Check[
    (NAV_ROUTE_STORAGE_WORDS <= FLASH_PAGE_LENGTH) ? 1 : -1];

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

static uint32 storage_slot_page(uint8 slot)
{
    return NAV_ROUTE_STORAGE_FIRST_PAGE + slot;
}

static bool storage_buffer_valid(uint8 max_route_len,
                                 uint8 *route_len,
                                 uint32 *sequence)
{
    uint32 stored_checksum;

    if (g_storage_buffer.image.magic != NAV_ROUTE_STORAGE_MAGIC ||
        g_storage_buffer.image.version != NAV_ROUTE_STORAGE_VERSION ||
        g_storage_buffer.image.route_len == 0u ||
        g_storage_buffer.image.route_len > NAV_RECORD_MAX_SEGMENTS ||
        g_storage_buffer.image.route_len > max_route_len ||
        g_storage_buffer.image.route[g_storage_buffer.image.route_len - 1u].action !=
            NAV_ACTION_STOP) {
        return false;
    }

    stored_checksum = g_storage_buffer.image.checksum;
    g_storage_buffer.image.checksum = 0u;

    if (stored_checksum !=
        storage_checksum(g_storage_buffer.words, NAV_ROUTE_STORAGE_WORDS)) {
        g_storage_buffer.image.checksum = stored_checksum;
        return false;
    }

    g_storage_buffer.image.checksum = stored_checksum;

    if (route_len != NULL) {
        *route_len = (uint8)g_storage_buffer.image.route_len;
    }

    if (sequence != NULL) {
        *sequence = g_storage_buffer.image.sequence;
    }

    return true;
}

static bool storage_read_slot(uint8 slot,
                              Nav_Segment_t *route,
                              uint8 max_route_len,
                              uint8 *route_len,
                              uint32 *sequence)
{
    uint8 stored_route_len = 0u;

    if (slot >= NAV_ROUTE_STORAGE_SLOT_COUNT ||
        max_route_len == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    flash_read_page(NAV_ROUTE_STORAGE_SECTOR,
                    storage_slot_page(slot),
                    g_storage_buffer.words,
                    NAV_ROUTE_STORAGE_WORDS);

    if (!storage_buffer_valid(max_route_len, &stored_route_len, sequence)) {
        return false;
    }

    if (route != NULL) {
        for (uint8 i = 0u; i < stored_route_len; i++) {
            route[i] = g_storage_buffer.image.route[i];
        }
    }

    if (route_len != NULL) {
        *route_len = stored_route_len;
    }

    return true;
}

static bool storage_find_latest_slots(uint8 *latest_slot,
                                      uint32 *latest_sequence,
                                      uint8 *previous_slot,
                                      uint32 *previous_sequence)
{
    bool have_latest = false;
    bool have_previous = false;
    uint8 route_len;
    uint32 sequence;

    *latest_slot = 0u;
    *latest_sequence = 0u;
    *previous_slot = NAV_ROUTE_STORAGE_SLOT_COUNT;
    *previous_sequence = 0u;

    for (uint8 slot = 0u; slot < NAV_ROUTE_STORAGE_SLOT_COUNT; slot++) {
        if (!storage_read_slot(slot,
                               NULL,
                               NAV_RECORD_MAX_SEGMENTS,
                               &route_len,
                               &sequence)) {
            continue;
        }

        if (!have_latest || sequence > *latest_sequence) {
            if (have_latest) {
                *previous_slot = *latest_slot;
                *previous_sequence = *latest_sequence;
                have_previous = true;
            }

            *latest_slot = slot;
            *latest_sequence = sequence;
            have_latest = true;
        } else if (!have_previous || sequence > *previous_sequence) {
            *previous_slot = slot;
            *previous_sequence = sequence;
            have_previous = true;
        }
    }

    (void)have_previous;
    return have_latest;
}

void nav_route_storage_init(void)
{
    storage_ensure_init();
}

bool nav_route_storage_save(const Nav_Segment_t *route, uint8 route_len)
{
    uint8 latest_slot = 0u;
    uint8 previous_slot = 0u;
    uint8 write_slot = 0u;
    uint32 latest_sequence = 0u;
    uint32 previous_sequence = 0u;
    uint32 sequence = 1u;

    if (route == NULL ||
        route_len == 0u ||
        route_len > NAV_RECORD_MAX_SEGMENTS ||
        route[route_len - 1u].action != NAV_ACTION_STOP ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    if (storage_find_latest_slots(&latest_slot,
                                  &latest_sequence,
                                  &previous_slot,
                                  &previous_sequence)) {
        write_slot = (uint8)((latest_slot + 1u) % NAV_ROUTE_STORAGE_SLOT_COUNT);
        sequence = latest_sequence + 1u;
    }

    memset(&g_storage_buffer, 0xFF, sizeof(g_storage_buffer));
    g_storage_buffer.image.magic = NAV_ROUTE_STORAGE_MAGIC;
    g_storage_buffer.image.version = NAV_ROUTE_STORAGE_VERSION;
    g_storage_buffer.image.sequence = sequence;
    g_storage_buffer.image.route_len = route_len;
    g_storage_buffer.image.checksum = 0u;

    for (uint8 i = 0u; i < route_len; i++) {
        g_storage_buffer.image.route[i] = route[i];
    }

    g_storage_buffer.image.checksum =
        storage_checksum(g_storage_buffer.words, NAV_ROUTE_STORAGE_WORDS);

    flash_write_page(NAV_ROUTE_STORAGE_SECTOR,
                     storage_slot_page(write_slot),
                     g_storage_buffer.words,
                     NAV_ROUTE_STORAGE_WORDS);

    return storage_read_slot(write_slot,
                             NULL,
                             NAV_RECORD_MAX_SEGMENTS,
                             NULL,
                             NULL);
}

bool nav_route_storage_load(Nav_Segment_t *route,
                            uint8 max_route_len,
                            uint8 *route_len)
{
    uint8 latest_slot = 0u;
    uint8 previous_slot = 0u;
    uint32 latest_sequence = 0u;
    uint32 previous_sequence = 0u;

    if (route == NULL ||
        route_len == NULL ||
        max_route_len == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    if (!storage_find_latest_slots(&latest_slot,
                                   &latest_sequence,
                                   &previous_slot,
                                   &previous_sequence)) {
        return false;
    }

    return storage_read_slot(latest_slot, route, max_route_len, route_len, NULL);
}

bool nav_route_storage_load_previous(Nav_Segment_t *route,
                                     uint8 max_route_len,
                                     uint8 *route_len)
{
    uint8 latest_slot = 0u;
    uint8 previous_slot = 0u;
    uint32 latest_sequence = 0u;
    uint32 previous_sequence = 0u;

    if (route == NULL ||
        route_len == NULL ||
        max_route_len == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    if (!storage_find_latest_slots(&latest_slot,
                                   &latest_sequence,
                                   &previous_slot,
                                   &previous_sequence) ||
        previous_slot >= NAV_ROUTE_STORAGE_SLOT_COUNT) {
        return false;
    }

    return storage_read_slot(previous_slot, route, max_route_len, route_len, NULL);
}
