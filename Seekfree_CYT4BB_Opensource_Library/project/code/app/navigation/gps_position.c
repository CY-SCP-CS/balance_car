#include "gps_position.h"

#include <math.h>
#include <string.h>

#define GPS_POSITION_EARTH_RADIUS_M         (6378137.0)
#define GPS_POSITION_MIN_SATELLITES         (6u)
#define GPS_POSITION_MIN_COURSE_SPEED_MPS   (0.30f)
#define GPS_POSITION_MAX_STEP_M             (5.0f)
#define GPS_POSITION_MAX_STEP_SPEED_MPS     (8.0f)
#define GPS_POSITION_FIX_TIMEOUT_MS         (600u)
#define GPS_POSITION_PI                     (3.14159265358979323846f)
#define GPS_POSITION_DEG_TO_RAD             (GPS_POSITION_PI / 180.0f)

typedef struct {
    bool initialized;
    bool origin_set;
    bool last_point_set;
    double origin_lat_rad;
    double origin_lon_rad;
    float last_x_m;
    float last_y_m;
    float path_m;
    uint32 last_fix_ms;
    Gps_Shared_Data_t shared;
} Gps_Position_Context_t;

static Gps_Position_Context_t g_gps_position;

static float gps_position_wrap_pi(float angle)
{
    while (angle > GPS_POSITION_PI) {
        angle -= 2.0f * GPS_POSITION_PI;
    }

    while (angle < -GPS_POSITION_PI) {
        angle += 2.0f * GPS_POSITION_PI;
    }

    return angle;
}

static double gps_position_signed_latitude(void)
{
    double latitude = gnss.latitude;

    if (gnss.ns == 'S') {
        latitude = -latitude;
    }

    return latitude;
}

static double gps_position_signed_longitude(void)
{
    double longitude = gnss.longitude;

    if (gnss.ew == 'W') {
        longitude = -longitude;
    }

    return longitude;
}

static bool gps_position_fix_usable(void)
{
    return gnss.state == 1u &&
           gnss.satellite_used >= GPS_POSITION_MIN_SATELLITES &&
           gnss.latitude != 0.0 &&
           gnss.longitude != 0.0;
}

static float gps_position_quality(void)
{
    float quality = (float)gnss.satellite_used / 12.0f;

    if (quality > 1.0f) {
        quality = 1.0f;
    }

    return quality;
}

static void gps_position_to_local(double latitude_deg,
                                  double longitude_deg,
                                  float *x_m,
                                  float *y_m)
{
    double latitude_rad = latitude_deg * GPS_POSITION_DEG_TO_RAD;
    double longitude_rad = longitude_deg * GPS_POSITION_DEG_TO_RAD;
    double d_lat = latitude_rad - g_gps_position.origin_lat_rad;
    double d_lon = longitude_rad - g_gps_position.origin_lon_rad;

    *x_m = (float)(d_lon * cos(g_gps_position.origin_lat_rad) *
                   GPS_POSITION_EARTH_RADIUS_M);
    *y_m = (float)(d_lat * GPS_POSITION_EARTH_RADIUS_M);
}

static float gps_position_distance(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;

    return sqrtf(dx * dx + dy * dy);
}

static bool gps_position_step_valid(float step_m, uint32 dt_ms)
{
    float max_step = GPS_POSITION_MAX_STEP_M;

    if (dt_ms > 0u) {
        float dt_s = (float)dt_ms * 0.001f;
        float speed_step = GPS_POSITION_MAX_STEP_SPEED_MPS * dt_s;

        if (speed_step > max_step) {
            max_step = speed_step;
        }
    }

    return step_m <= max_step;
}

static void gps_position_publish_timeout(uint32 now_ms)
{
    if (g_gps_position.shared.valid == 0u) {
        return;
    }

    if ((uint32)(now_ms - g_gps_position.last_fix_ms) <= GPS_POSITION_FIX_TIMEOUT_MS) {
        return;
    }

    g_gps_position.shared.valid = 0u;
    g_gps_position.shared.time_ms = now_ms;
    gps_shared_publish(&g_gps_position.shared);
}

static void gps_position_accept_fix(uint32 now_ms)
{
    double latitude_deg = gps_position_signed_latitude();
    double longitude_deg = gps_position_signed_longitude();
    float x_m = 0.0f;
    float y_m = 0.0f;
    float step_m = 0.0f;
    uint32 dt_ms = now_ms - g_gps_position.last_fix_ms;
    bool step_valid = true;

    if (!g_gps_position.origin_set) {
        g_gps_position.origin_lat_rad = latitude_deg * GPS_POSITION_DEG_TO_RAD;
        g_gps_position.origin_lon_rad = longitude_deg * GPS_POSITION_DEG_TO_RAD;
        g_gps_position.origin_set = true;
        g_gps_position.last_point_set = false;
        g_gps_position.path_m = 0.0f;
    }

    gps_position_to_local(latitude_deg, longitude_deg, &x_m, &y_m);

    if (g_gps_position.last_point_set) {
        step_m = gps_position_distance(g_gps_position.last_x_m,
                                       g_gps_position.last_y_m,
                                       x_m,
                                       y_m);
        step_valid = gps_position_step_valid(step_m, dt_ms);
    }

    memset(&g_gps_position.shared, 0, sizeof(g_gps_position.shared));
    g_gps_position.shared.magic = GPS_SHARED_MAGIC;
    g_gps_position.shared.time_ms = now_ms;
    g_gps_position.shared.last_fix_ms = now_ms;
    g_gps_position.shared.origin_set = g_gps_position.origin_set ? 1u : 0u;
    g_gps_position.shared.satellite_used = gnss.satellite_used;
    g_gps_position.shared.latitude_deg = (float)latitude_deg;
    g_gps_position.shared.longitude_deg = (float)longitude_deg;
    g_gps_position.shared.height_m = gnss.height;
    g_gps_position.shared.x_m = x_m;
    g_gps_position.shared.y_m = y_m;
    g_gps_position.shared.speed_mps = gnss.speed / 3.6f;
    g_gps_position.shared.quality = gps_position_quality();
    g_gps_position.shared.distance_from_origin_m =
        gps_position_distance(0.0f, 0.0f, x_m, y_m);

    if (gnss.speed / 3.6f >= GPS_POSITION_MIN_COURSE_SPEED_MPS) {
        g_gps_position.shared.course_valid = 1u;
        g_gps_position.shared.course_rad =
            gps_position_wrap_pi(gnss.direction * GPS_POSITION_DEG_TO_RAD);
    }

    if (gnss.antenna_direction_state == 1u) {
        g_gps_position.shared.antenna_yaw_valid = 1u;
        g_gps_position.shared.antenna_yaw_rad =
            gps_position_wrap_pi(gnss.antenna_direction * GPS_POSITION_DEG_TO_RAD);
    }

    if (step_valid) {
        if (g_gps_position.last_point_set) {
            g_gps_position.path_m += step_m;
        }

        g_gps_position.last_x_m = x_m;
        g_gps_position.last_y_m = y_m;
        g_gps_position.last_point_set = true;
        g_gps_position.last_fix_ms = now_ms;
        g_gps_position.shared.valid = 1u;
        g_gps_position.shared.path_m = g_gps_position.path_m;
    } else {
        g_gps_position.shared.valid = 0u;
        g_gps_position.shared.path_m = g_gps_position.path_m;
    }

    gps_shared_publish(&g_gps_position.shared);
}

void gps_position_init(gps_device_enum gps_device)
{
    memset(&g_gps_position, 0, sizeof(g_gps_position));
    gps_shared_clear();
    gnss_init(gps_device);
    g_gps_position.initialized = true;
}

void gps_position_update(uint32 now_ms)
{
    if (!g_gps_position.initialized) {
        return;
    }

    if (gnss_flag != 0u) {
        gnss_flag = 0u;

        if (gnss_data_parse() == 0u && gps_position_fix_usable()) {
            gps_position_accept_fix(now_ms);
        } else {
            gps_position_publish_timeout(now_ms);
        }
    } else {
        gps_position_publish_timeout(now_ms);
    }
}

Gps_Position_Debug_t gps_position_get_debug(void)
{
    Gps_Position_Debug_t debug;

    memset(&debug, 0, sizeof(debug));
    debug.initialized = g_gps_position.initialized;
    debug.origin_set = g_gps_position.origin_set;
    debug.fix_valid = g_gps_position.shared.valid != 0u;
    debug.satellite_used = g_gps_position.shared.satellite_used;
    debug.x_m = g_gps_position.shared.x_m;
    debug.y_m = g_gps_position.shared.y_m;
    debug.path_m = g_gps_position.shared.path_m;
    debug.speed_mps = g_gps_position.shared.speed_mps;
    debug.course_rad = g_gps_position.shared.course_rad;
    debug.quality = g_gps_position.shared.quality;

    return debug;
}
