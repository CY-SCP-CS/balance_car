#include "nav_route_storage.h"

#include <stddef.h>
#include <string.h>

#include "zf_driver_flash.h"

#define NAV_ROUTE_STORAGE_MAGIC    0x4E415652u
#define NAV_ROUTE_STORAGE_VERSION  7u
#define NAV_ROUTE_STORAGE_SLOT_COUNT  3u
#define NAV_ROUTE_STORAGE_FIRST_PAGE  (FLASH_PAGE_NUM - NAV_ROUTE_STORAGE_SLOT_COUNT)
#define NAV_ROUTE_STORAGE_SECTOR   0u

typedef struct {
    uint32 magic;
    uint32 version;
    uint32 sequence;
    uint32 keypoint_count;
    uint32 checksum;
    Nav_Keypoint_t keypoints[NAV_RECORD_MAX_KEYPOINTS];
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

static bool storage_buffer_valid(uint8 max_keypoints,
                                 uint8 *keypoint_count,
                                 uint32 *sequence)
{
    uint32 stored_checksum;

    if (g_storage_buffer.image.magic != NAV_ROUTE_STORAGE_MAGIC ||
        g_storage_buffer.image.version != NAV_ROUTE_STORAGE_VERSION ||
        g_storage_buffer.image.keypoint_count < 2u ||
        g_storage_buffer.image.keypoint_count > NAV_RECORD_MAX_KEYPOINTS ||
        g_storage_buffer.image.keypoint_count > max_keypoints) {
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

    if (keypoint_count != NULL) {
        *keypoint_count = (uint8)g_storage_buffer.image.keypoint_count;
    }

    if (sequence != NULL) {
        *sequence = g_storage_buffer.image.sequence;
    }

    return true;
}

static bool storage_read_slot(uint8 slot,
                              Nav_Keypoint_t *keypoints,
                              uint8 max_keypoints,
                              uint8 *keypoint_count,
                              uint32 *sequence)
{
    uint8 stored_keypoint_count = 0u;

    if (slot >= NAV_ROUTE_STORAGE_SLOT_COUNT ||
        max_keypoints == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    flash_read_page(NAV_ROUTE_STORAGE_SECTOR,
                    storage_slot_page(slot),
                    g_storage_buffer.words,
                    NAV_ROUTE_STORAGE_WORDS);

    if (!storage_buffer_valid(max_keypoints, &stored_keypoint_count, sequence)) {
        return false;
    }

    if (keypoints != NULL) {
        for (uint8 i = 0u; i < stored_keypoint_count; i++) {
            keypoints[i] = g_storage_buffer.image.keypoints[i];
        }
    }

    if (keypoint_count != NULL) {
        *keypoint_count = stored_keypoint_count;
    }

    return true;
}

static bool storage_find_latest_slot(uint8 *latest_slot,
                                     uint32 *latest_sequence)
{
    bool have_latest = false;
    uint8 keypoint_count;
    uint32 sequence;

    *latest_slot = 0u;
    *latest_sequence = 0u;

    for (uint8 slot = 0u; slot < NAV_ROUTE_STORAGE_SLOT_COUNT; slot++) {
        if (!storage_read_slot(slot,
                               NULL,
                               NAV_RECORD_MAX_KEYPOINTS,
                               &keypoint_count,
                               &sequence)) {
            continue;
        }

        if (!have_latest || sequence > *latest_sequence) {
            *latest_slot = slot;
            *latest_sequence = sequence;
            have_latest = true;
        }
    }

    return have_latest;
}

static bool storage_find_history_slot(uint8 history_index, uint8 *history_slot)
{
    bool used[NAV_ROUTE_STORAGE_SLOT_COUNT] = { false };
    uint8 selected_slot = 0u;

    if (history_slot == NULL ||
        history_index >= NAV_ROUTE_STORAGE_SLOT_COUNT) {
        return false;
    }

    for (uint8 rank = 0u; rank <= history_index; rank++) {
        bool have_best = false;
        uint8 best_slot = 0u;
        uint8 keypoint_count;
        uint32 best_sequence = 0u;
        uint32 sequence;

        for (uint8 slot = 0u; slot < NAV_ROUTE_STORAGE_SLOT_COUNT; slot++) {
            if (used[slot]) {
                continue;
            }

            if (!storage_read_slot(slot,
                                   NULL,
                                   NAV_RECORD_MAX_KEYPOINTS,
                                   &keypoint_count,
                                   &sequence)) {
                continue;
            }

            if (!have_best || sequence > best_sequence) {
                best_slot = slot;
                best_sequence = sequence;
                have_best = true;
            }
        }

        if (!have_best) {
            return false;
        }

        used[best_slot] = true;
        selected_slot = best_slot;
    }

    *history_slot = selected_slot;
    return true;
}

void nav_route_storage_init(void)
{
    storage_ensure_init();
}

bool nav_route_storage_save(const Nav_Keypoint_t *keypoints,
                            uint8 keypoint_count)
{
    uint8 latest_slot = 0u;
    uint8 write_slot = 0u;
    uint32 latest_sequence = 0u;
    uint32 sequence = 1u;

    if (keypoints == NULL ||
        keypoint_count < 2u ||
        keypoint_count > NAV_RECORD_MAX_KEYPOINTS ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    if (storage_find_latest_slot(&latest_slot, &latest_sequence)) {
        write_slot = (uint8)((latest_slot + 1u) % NAV_ROUTE_STORAGE_SLOT_COUNT);
        sequence = latest_sequence + 1u;
    }

    memset(&g_storage_buffer, 0xFF, sizeof(g_storage_buffer));
    g_storage_buffer.image.magic = NAV_ROUTE_STORAGE_MAGIC;
    g_storage_buffer.image.version = NAV_ROUTE_STORAGE_VERSION;
    g_storage_buffer.image.sequence = sequence;
    g_storage_buffer.image.keypoint_count = keypoint_count;
    g_storage_buffer.image.checksum = 0u;

    for (uint8 i = 0u; i < keypoint_count; i++) {
        g_storage_buffer.image.keypoints[i] = keypoints[i];
    }

    g_storage_buffer.image.checksum =
        storage_checksum(g_storage_buffer.words, NAV_ROUTE_STORAGE_WORDS);

    flash_write_page(NAV_ROUTE_STORAGE_SECTOR,
                     storage_slot_page(write_slot),
                     g_storage_buffer.words,
                     NAV_ROUTE_STORAGE_WORDS);

    return storage_read_slot(write_slot,
                             NULL,
                             NAV_RECORD_MAX_KEYPOINTS,
                             NULL,
                             NULL);
}

bool nav_route_storage_load_history(Nav_Keypoint_t *keypoints,
                                    uint8 max_keypoints,
                                    uint8 *keypoint_count,
                                    uint8 history_index)
{
    uint8 history_slot = 0u;

    if (keypoints == NULL ||
        keypoint_count == NULL ||
        max_keypoints == 0u ||
        NAV_ROUTE_STORAGE_WORDS > FLASH_PAGE_LENGTH) {
        return false;
    }

    storage_ensure_init();

    if (!storage_find_history_slot(history_index, &history_slot)) {
        return false;
    }

    return storage_read_slot(history_slot,
                             keypoints,
                             max_keypoints,
                             keypoint_count,
                             NULL);
}

bool nav_route_storage_load(Nav_Keypoint_t *keypoints,
                            uint8 max_keypoints,
                            uint8 *keypoint_count)
{
    return nav_route_storage_load_history(keypoints,
                                          max_keypoints,
                                          keypoint_count,
                                          0u);
}

bool nav_route_storage_load_previous(Nav_Keypoint_t *keypoints,
                                     uint8 max_keypoints,
                                     uint8 *keypoint_count)
{
    return nav_route_storage_load_history(keypoints,
                                          max_keypoints,
                                          keypoint_count,
                                          1u);
}
