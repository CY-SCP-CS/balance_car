#include "nav_adapter.h"
#include "nav_internal.h"
#include "gps_shared.h"

#include "../robot_control/robot_control.h"

#define NAV_GPS_DISTANCE_GAIN              (0.02f)
#define NAV_GPS_COURSE_YAW_GAIN            (0.01f)
#define NAV_GPS_ANTENNA_YAW_GAIN           (0.04f)
#define NAV_GPS_MAX_DISTANCE_ERROR_M       (2.0f)
#define NAV_GPS_MAX_YAW_CORRECTION_RAD     (25.0f * NAV_DEG_TO_RAD)
#define NAV_GPS_YAW_SIGN                   (1.0f)

static bool g_nav_odom_hold = false;
static bool g_nav_odom_resync = false;

static float nav_limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

void nav_adapter_set_odom_hold(bool hold)
{
    if (g_nav_odom_hold != hold) {
        g_nav_odom_hold = hold;
        g_nav_odom_resync = true;
    }
}

bool nav_adapter_is_odom_held(void)
{
    return g_nav_odom_hold;
}

void nav_adapter_reset_odom_base(void)
{
    g_nav_odom_resync = true;
}

void nav_input_update_from_ctrl(Nav_Input_t *input, const Ctrl_Input_t *ctrl)
{
    static bool odom_set = false;
    static float last_odom_distance = 0.0f;
    static float last_odom_x = 0.0f;
    static float last_odom_y = 0.0f;
    static float nav_x = 0.0f;
    static float nav_y = 0.0f;
    static float fused_distance = 0.0f;
    static bool gps_distance_anchor_set = false;
    static float gps_distance_offset = 0.0f;
    static bool gps_yaw_anchor_set = false;
    static float gps_yaw_offset = 0.0f;
    static float gps_yaw_correction = 0.0f;
    static uint32 last_gps_sequence = 0u;
    float odom_yaw;
    float odom_distance;
    float odom_x;
    float odom_y;
    Gps_Shared_Data_t gps;

    if (input == NULL || ctrl == NULL) {
        return;
    }

    input->time_ms += NAV_LOOP_DT_MS;

    odom_distance = robot_control_get_distance();
    odom_x = robot_control_get_x();
    odom_y = robot_control_get_y();
    odom_yaw = robot_control_get_yaw();
    if (!odom_set) {
        last_odom_distance = odom_distance;
        last_odom_x = odom_x;
        last_odom_y = odom_y;
        nav_x = odom_x;
        nav_y = odom_y;
        fused_distance = odom_distance;
        odom_set = true;
        g_nav_odom_resync = false;
    } else if (g_nav_odom_resync) {
        last_odom_distance = odom_distance;
        last_odom_x = odom_x;
        last_odom_y = odom_y;
        gps_distance_anchor_set = false;
        g_nav_odom_resync = false;
    } else if (g_nav_odom_hold) {
        last_odom_distance = odom_distance;
        last_odom_x = odom_x;
        last_odom_y = odom_y;
    } else {
        fused_distance += odom_distance - last_odom_distance;
        nav_x += odom_x - last_odom_x;
        nav_y += odom_y - last_odom_y;
        last_odom_distance = odom_distance;
        last_odom_x = odom_x;
        last_odom_y = odom_y;
    }

    if (g_nav_odom_hold) {
        gps_distance_anchor_set = false;
    } else if (gps_shared_read(&gps) && gps.valid != 0u && gps.origin_set != 0u) {
        bool new_gps_sample = gps.sequence != last_gps_sequence;

        if (gps.sequence < last_gps_sequence) {
            gps_distance_anchor_set = false;
            gps_yaw_anchor_set = false;
            gps_yaw_correction = 0.0f;
            new_gps_sample = true;
        }

        if (new_gps_sample) {
            if (!gps_distance_anchor_set) {
                gps_distance_offset = fused_distance - gps.path_m;
                gps_distance_anchor_set = true;
            } else {
                float gps_distance = gps.path_m + gps_distance_offset;
                float distance_error = gps_distance - fused_distance;

                if (distance_error < NAV_GPS_MAX_DISTANCE_ERROR_M &&
                    distance_error > -NAV_GPS_MAX_DISTANCE_ERROR_M) {
                    fused_distance += NAV_GPS_DISTANCE_GAIN * distance_error;
                }
            }

            if (gps.antenna_yaw_valid != 0u || gps.course_valid != 0u) {
                float gps_yaw = (gps.antenna_yaw_valid != 0u) ?
                                gps.antenna_yaw_rad : gps.course_rad;
                float yaw_gain = (gps.antenna_yaw_valid != 0u) ?
                                 NAV_GPS_ANTENNA_YAW_GAIN :
                                 NAV_GPS_COURSE_YAW_GAIN;

                gps_yaw = nav_wrap_pi(NAV_GPS_YAW_SIGN * gps_yaw);

                if (!gps_yaw_anchor_set) {
                    gps_yaw_offset = nav_wrap_pi(odom_yaw - gps_yaw);
                    gps_yaw_anchor_set = true;
                } else {
                    float gps_yaw_relative = nav_wrap_pi(gps_yaw + gps_yaw_offset);
                    float yaw_error = nav_wrap_pi(gps_yaw_relative - odom_yaw);

                    gps_yaw_correction = nav_wrap_pi(gps_yaw_correction +
                                                     yaw_gain * yaw_error);
                    gps_yaw_correction =
                        nav_limit(gps_yaw_correction,
                                  NAV_GPS_MAX_YAW_CORRECTION_RAD);
                }
            }

            last_gps_sequence = gps.sequence;
        }
    }

    input->yaw_rad = nav_wrap_pi(odom_yaw + gps_yaw_correction);
    input->x_m = nav_x;
    input->y_m = nav_y;
    input->distance_m = fused_distance;
    input->speed_mps = robot_control_get_speed_mps();

    /* Vision updates landmark / obstacle fields after this update. */
}

void nav_apply_ctrl(Ctrl_Input_t *ctrl, const Nav_Output_t *nav)
{
    if (ctrl == NULL || nav == NULL) {
        return;
    }

    ctrl->velocity_cmd = nav->velocity_cmd;
    ctrl->steering_cmd = nav->steering_cmd;
    ctrl->yaw_target_valid = nav->target_yaw_valid;
    ctrl->yaw_target_rad = nav->target_yaw_rad;
}
