#include "nav_route_record.h"
#include "nav_internal.h"

#include <math.h>
#include <stddef.h>

#define NAV_RECORD_STRAIGHT_SPEED  0.18f
#define NAV_RECORD_TURN_SPEED      0.0f
#define NAV_RECORD_MIN_DISTANCE_M  0.06f
#define NAV_RECORD_MIN_TURN_RAD    (12.0f * NAV_DEG_TO_RAD)
#define NAV_RECORD_STRAIGHT_TIMEOUT_BASE_MS  2000u
#define NAV_RECORD_STRAIGHT_TIMEOUT_PER_M_MS 7000u
#define NAV_RECORD_TURN_TIMEOUT_BASE_MS      1500u
#define NAV_RECORD_TURN_TIMEOUT_PER_DEG_MS   35u
#define NAV_RECORD_TIMEOUT_MAX_MS            15000u

static Nav_Keypoint_t g_record_keypoints[NAV_RECORD_MAX_KEYPOINTS];
static Nav_Segment_t g_record_route[NAV_RECORD_MAX_SEGMENTS];
static Nav_Route_Record_State_t g_record_state;

static uint32 limit_timeout(uint32 timeout_ms)
{
    return timeout_ms > NAV_RECORD_TIMEOUT_MAX_MS ?
        NAV_RECORD_TIMEOUT_MAX_MS : timeout_ms;
}

static bool append_record_segment(Nav_Action_t action,
                                  float target_distance_m,
                                  float target_yaw_deg,
                                  float target_speed,
                                  uint32 timeout_ms)
{
    if (g_record_state.segment_count >= NAV_RECORD_MAX_SEGMENTS) {
        g_record_state.overflow = true;
        return false;
    }

    g_record_route[g_record_state.segment_count].action = action;
    g_record_route[g_record_state.segment_count].target_distance_m = target_distance_m;
    g_record_route[g_record_state.segment_count].target_yaw_deg = target_yaw_deg;
    g_record_route[g_record_state.segment_count].target_speed = target_speed;
    g_record_route[g_record_state.segment_count].timeout_ms = timeout_ms;
    g_record_route[g_record_state.segment_count].landmark = NAV_LANDMARK_NONE;
    g_record_state.segment_count++;

    return true;
}

static bool build_recorded_route(void)
{
    g_record_state.segment_count = 0u;
    g_record_state.route_ready = false;

    if (g_record_state.keypoint_count < 2u) {
        return false;
    }

    for (uint8 i = 1u; i < g_record_state.keypoint_count; i++) {
        const Nav_Keypoint_t *prev = &g_record_keypoints[i - 1u];
        const Nav_Keypoint_t *cur = &g_record_keypoints[i];
        float delta_distance = cur->distance_m - prev->distance_m;
        float delta_yaw = nav_wrap_pi(cur->yaw_rad - prev->yaw_rad);

        if (delta_distance < 0.0f) {
            delta_distance = 0.0f;
        }

        if (delta_distance >= NAV_RECORD_MIN_DISTANCE_M) {
            float turn_deg = delta_yaw * NAV_RAD_TO_DEG;
            uint32 timeout_ms = NAV_RECORD_STRAIGHT_TIMEOUT_BASE_MS +
                (uint32)(delta_distance * (float)NAV_RECORD_STRAIGHT_TIMEOUT_PER_M_MS) +
                (uint32)(fabsf(turn_deg) * (float)NAV_RECORD_TURN_TIMEOUT_PER_DEG_MS);

            if (!append_record_segment(NAV_ACTION_GO_STRAIGHT,
                                       delta_distance,
                                       turn_deg,
                                       NAV_RECORD_STRAIGHT_SPEED,
                                       limit_timeout(timeout_ms))) {
                return false;
            }
        } else if (fabsf(delta_yaw) >= NAV_RECORD_MIN_TURN_RAD) {
            float turn_deg = delta_yaw * NAV_RAD_TO_DEG;
            uint32 timeout_ms = NAV_RECORD_TURN_TIMEOUT_BASE_MS +
                (uint32)(fabsf(turn_deg) * (float)NAV_RECORD_TURN_TIMEOUT_PER_DEG_MS);

            if (!append_record_segment(NAV_ACTION_TURN,
                                       0.0f,
                                       turn_deg,
                                       NAV_RECORD_TURN_SPEED,
                                       limit_timeout(timeout_ms))) {
                return false;
            }
        }
    }

    if (g_record_state.segment_count == 0u) {
        return false;
    }

    if (!append_record_segment(NAV_ACTION_STOP,
                               0.0f,
                               0.0f,
                               0.0f,
                               0u)) {
        return false;
    }

    g_record_state.route_ready = true;
    return true;
}

void nav_route_record_notify_navigation_stopped(bool finished,
                                                bool safety_stop)
{
    if (g_record_state.mode == NAV_ROUTE_REPLAYING &&
        (finished || safety_stop)) {
        g_record_state.mode = g_record_state.route_ready ?
            NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    }
}

bool nav_route_record_start(const Nav_Input_t *input)
{
    if (input == NULL) {
        return false;
    }

    nav_stop();
    g_record_state.mode = NAV_ROUTE_RECORDING;
    g_record_state.keypoint_count = 1u;
    g_record_state.segment_count = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;

    g_record_keypoints[0].distance_m = input->distance_m;
    g_record_keypoints[0].yaw_rad = input->yaw_rad;
    g_record_keypoints[0].time_ms = input->time_ms;

    return true;
}

bool nav_route_record_keypoint(const Nav_Input_t *input)
{
    if (input == NULL || g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (g_record_state.keypoint_count >= NAV_RECORD_MAX_KEYPOINTS) {
        g_record_state.overflow = true;
        return false;
    }

    uint8 index = g_record_state.keypoint_count;
    g_record_keypoints[index].distance_m = input->distance_m;
    g_record_keypoints[index].yaw_rad = input->yaw_rad;
    g_record_keypoints[index].time_ms = input->time_ms;
    g_record_state.keypoint_count++;

    return true;
}

bool nav_route_record_finish(void)
{
    if (g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (!build_recorded_route()) {
        g_record_state.mode = NAV_ROUTE_IDLE;
        g_record_state.route_ready = false;
        return false;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    return true;
}

void nav_route_record_reset(void)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = 0u;
    g_record_state.segment_count = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
}

bool nav_route_replay_start(const Nav_Input_t *input)
{
    if (input == NULL || !g_record_state.route_ready ||
        g_record_state.segment_count == 0u) {
        return false;
    }

    nav_set_route(g_record_route, g_record_state.segment_count);
    g_record_state.mode = NAV_ROUTE_REPLAYING;
    nav_start(input);

    return true;
}

void nav_route_replay_stop(void)
{
    nav_stop();
    g_record_state.mode = g_record_state.route_ready ?
        NAV_ROUTE_READY : NAV_ROUTE_IDLE;
}

Nav_Route_Record_State_t nav_route_record_get_state(void)
{
    return g_record_state;
}

const Nav_Segment_t *nav_route_recorded_route(uint8 *route_len)
{
    if (route_len != NULL) {
        *route_len = g_record_state.segment_count;
    }

    return g_record_route;
}
