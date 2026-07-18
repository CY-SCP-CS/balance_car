#include "gps_shared.h"

#include <string.h>

#include "zf_common_headfile.h"

#define GPS_SHARED_CACHE_BYTES  (sizeof(Gps_Shared_Data_t))

#if defined(__ICCARM__)
#pragma location = 0x28006C80
__no_init volatile Gps_Shared_Data_t g_gps_shared;
#else
volatile Gps_Shared_Data_t g_gps_shared;
#endif

static uint32 g_publish_sequence = 0u;

static void gps_shared_clean_cache(void)
{
    SCB_CleanDCache_by_Addr((uint32 *)&g_gps_shared, GPS_SHARED_CACHE_BYTES);
}

static void gps_shared_invalidate_cache(void)
{
    SCB_InvalidateDCache_by_Addr((uint32 *)&g_gps_shared, GPS_SHARED_CACHE_BYTES);
}

void gps_shared_clear(void)
{
    Gps_Shared_Data_t data;

    memset(&data, 0, sizeof(data));
    data.magic = GPS_SHARED_MAGIC;

    gps_shared_publish(&data);
}

void gps_shared_publish(const Gps_Shared_Data_t *data)
{
    Gps_Shared_Data_t copy;
    uint32 next_sequence;

    if (data == NULL) {
        return;
    }

    copy = *data;
    copy.magic = GPS_SHARED_MAGIC;

    next_sequence = g_publish_sequence + 2u;
    if (next_sequence < 2u) {
        next_sequence = 2u;
    }
    g_publish_sequence = next_sequence;

    g_gps_shared.sequence = next_sequence | 1u;
    gps_shared_clean_cache();

    copy.sequence = next_sequence;
    g_gps_shared = copy;
    gps_shared_clean_cache();
}

bool gps_shared_read(Gps_Shared_Data_t *out)
{
    Gps_Shared_Data_t snapshot;
    uint32 sequence_before;
    uint32 sequence_after;

    if (out == NULL) {
        return false;
    }

    gps_shared_invalidate_cache();
    sequence_before = g_gps_shared.sequence;
    if ((sequence_before & 1u) != 0u) {
        return false;
    }

    snapshot = g_gps_shared;

    gps_shared_invalidate_cache();
    sequence_after = g_gps_shared.sequence;
    if (sequence_before != sequence_after ||
        (sequence_after & 1u) != 0u ||
        snapshot.magic != GPS_SHARED_MAGIC) {
        return false;
    }

    *out = snapshot;
    return true;
}
