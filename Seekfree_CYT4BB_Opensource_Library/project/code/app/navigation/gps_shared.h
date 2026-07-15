#ifndef APP_NAVIGATION_GPS_SHARED_H
#define APP_NAVIGATION_GPS_SHARED_H

#include <stdbool.h>

#include "zf_common_typedef.h"

#define GPS_SHARED_MAGIC        (0x47505331u)   /* GPS1 */
#define GPS_SHARED_ADDRESS      (0x28006C80u)

typedef struct {
    uint32 magic;
    uint32 sequence;
    uint32 time_ms;
    uint32 last_fix_ms;

    uint8 valid;
    uint8 origin_set;
    uint8 satellite_used;
    uint8 course_valid;
    uint8 antenna_yaw_valid;
    uint8 reserved0[3];

    float latitude_deg;
    float longitude_deg;
    float height_m;
    float x_m;
    float y_m;
    float path_m;
    float speed_mps;
    float course_rad;
    float antenna_yaw_rad;
    float distance_from_origin_m;
    float quality;
    float reserved1[7];
} Gps_Shared_Data_t;

void gps_shared_clear(void);
void gps_shared_publish(const Gps_Shared_Data_t *data);
bool gps_shared_read(Gps_Shared_Data_t *out);

#endif
