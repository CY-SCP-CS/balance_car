#ifndef APP_NAVIGATION_GPS_POSITION_H
#define APP_NAVIGATION_GPS_POSITION_H

#include <stdbool.h>

#include "gps_shared.h"
#include "zf_device_gnss.h"

typedef struct {
    bool initialized;
    bool origin_set;
    bool fix_valid;
    uint8 satellite_used;
    float x_m;
    float y_m;
    float path_m;
    float speed_mps;
    float course_rad;
    float quality;
} Gps_Position_Debug_t;

void gps_position_init(gps_device_enum gps_device);
void gps_position_update(uint32 now_ms);
Gps_Position_Debug_t gps_position_get_debug(void);

#endif
