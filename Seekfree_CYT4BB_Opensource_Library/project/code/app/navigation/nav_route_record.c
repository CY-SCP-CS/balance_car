#include "nav_route_record.h"
#include "nav_internal.h"
#include "nav_route_storage.h"

#include <math.h>
#include <stddef.h>

#include "../../common/utils.h"

#define NAV_RECORD_STRAIGHT_SPEED  0.18f
#define NAV_RECORD_TURN_SPEED      0.0f
#define NAV_RECORD_MIN_DISTANCE_M  0.06f
#define NAV_RECORD_TIMEOUT_MAX_MS            15000u

static Nav_Keypoint_t g_record_keypoints[NAV_RECORD_MAX_KEYPOINTS];
static Nav_Route_Record_State_t g_record_state;
static float g_replay_start_distance;
static float g_replay_start_yaw;
static uint32 g_replay_start_time;

static float keypoint_distance_from_start(uint8 index)
{
    float distance;

    if (g_record_state.keypoint_count == 0u) {
        return 0.0f;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    distance = g_record_keypoints[index].distance_m -
               g_record_keypoints[0].distance_m;

    return distance > 0.0f ? distance : 0.0f;
}

static float keypoint_yaw_delta_from_start(uint8 index)
{
    float yaw_delta = 0.0f;

    if (g_record_state.keypoint_count == 0u) {
        return 0.0f;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    for (uint8 i = 1u; i <= index; i++) {
        const Nav_Keypoint_t *prev = &g_record_keypoints[i - 1u];
        const Nav_Keypoint_t *cur = &g_record_keypoints[i];
        yaw_delta += nav_wrap_pi(cur->yaw_rad - prev->yaw_rad);
    }

    return yaw_delta;
}

static uint32 keypoint_elapsed_from_start(uint8 index)
{
    if (g_record_state.keypoint_count == 0u) {
        return 0u;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    return (uint32)(g_record_keypoints[index].time_ms -
                    g_record_keypoints[0].time_ms);
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
    g_record_state.replay_index = 0u;
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
    bool saved;

    if (g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (g_record_state.keypoint_count < 2u) {
        g_record_state.mode = NAV_ROUTE_IDLE;
        g_record_state.route_ready = false;
        return false;
    }

    saved = nav_route_storage_save(g_record_keypoints,
                                   g_record_state.keypoint_count);

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.route_ready = true;
    g_record_state.replay_index = 0u;
    return saved;
}

void nav_route_record_reset(void)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = 0u;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
}

static bool set_loaded_keypoints_ready(uint8 keypoint_count)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = keypoint_count;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;

    if (g_record_state.keypoint_count < 2u) {
        g_record_state.keypoint_count = 0u;
        return false;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.route_ready = true;
    return true;
}

bool nav_route_record_load_saved_history(uint8 history_index)
{
    uint8 keypoint_count = 0u;

    if (!nav_route_storage_load_history(g_record_keypoints,
                                        NAV_RECORD_MAX_KEYPOINTS,
                                        &keypoint_count,
                                        history_index)) {
        return false;
    }

    return set_loaded_keypoints_ready(keypoint_count);
}

bool nav_route_record_load_saved(void)
{
    return nav_route_record_load_saved_history(0u);
}

bool nav_route_record_load_previous_saved(void)
{
    return nav_route_record_load_saved_history(1u);
}

bool nav_route_replay_start(const Nav_Input_t *input)
{
    if (input == NULL || !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return false;
    }

    nav_stop();
    g_record_state.mode = NAV_ROUTE_REPLAYING;
    g_record_state.replay_index = 1u;
    g_replay_start_distance = input->distance_m;
    g_replay_start_yaw = input->yaw_rad;
    g_replay_start_time = input->time_ms;

    return true;
}

Nav_Output_t nav_route_replay_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {
        0.0f, 0.0f, false, false, NAV_REGION_NONE, false, false
    };
    Nav_Config_t cfg;
    float traveled;

    if (input == NULL ||
        g_record_state.mode != NAV_ROUTE_REPLAYING ||
        !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return out;
    }

    if (input->obstacle_close) {
        g_record_state.mode = NAV_ROUTE_READY;
        g_record_state.replay_index = 0u;
        out.safety_stop = true;
        return out;
    }

    cfg = nav_get_config();
    traveled = input->distance_m - g_replay_start_distance;
    if (traveled < 0.0f) {
        traveled = 0.0f;
    }

    while (g_record_state.replay_index < g_record_state.keypoint_count) {
        uint8 prev_index = (uint8)(g_record_state.replay_index - 1u);
        uint8 cur_index = g_record_state.replay_index;
        float prev_distance = keypoint_distance_from_start(prev_index);
        float target_distance = keypoint_distance_from_start(cur_index);
        float segment_distance = target_distance - prev_distance;
        float prev_yaw_delta = keypoint_yaw_delta_from_start(prev_index);
        float target_yaw_delta = keypoint_yaw_delta_from_start(cur_index);
        float segment_yaw_delta = target_yaw_delta - prev_yaw_delta;
        float progress = 1.0f;
        float target_yaw;
        float yaw_error;
        bool timed_out;
        bool arrived;
        uint32 target_elapsed = keypoint_elapsed_from_start(cur_index);
        uint32 replay_elapsed = (uint32)(input->time_ms - g_replay_start_time);

        if (segment_distance > 0.0f) {
            progress = clamp((traveled - prev_distance) / segment_distance,
                             0.0f,
                             1.0f);
        }

        target_yaw = g_replay_start_yaw +
                     prev_yaw_delta +
                     segment_yaw_delta * progress;
        yaw_error = nav_wrap_pi(target_yaw - input->yaw_rad);
        timed_out = target_elapsed > 0u &&
                    replay_elapsed > target_elapsed &&
                    (uint32)(replay_elapsed - target_elapsed) >
                    NAV_RECORD_TIMEOUT_MAX_MS;

        if (segment_distance >= NAV_RECORD_MIN_DISTANCE_M) {
            out.velocity_cmd = NAV_RECORD_STRAIGHT_SPEED;
            out.steering_cmd = cfg.yaw_kp * yaw_error;
            arrived = traveled >= target_distance - cfg.distance_tolerance_m;
        } else {
            out.velocity_cmd = NAV_RECORD_TURN_SPEED;
            out.steering_cmd = cfg.turn_kp * yaw_error;
            arrived = fabsf(segment_yaw_delta) <= cfg.yaw_tolerance_rad ||
                      fabsf(yaw_error) <= cfg.yaw_tolerance_rad;
        }

        if (!arrived && !timed_out) {
            out.region = NAV_REGION_NORMAL;
            out.steering_cmd = clamp(out.steering_cmd,
                                     -cfg.steering_limit,
                                     cfg.steering_limit);
            return out;
        }

        g_record_state.replay_index++;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.replay_index = 0u;
    out.finished = true;
    return out;
}

void nav_route_replay_stop(void)
{
    nav_stop();
    g_record_state.mode = g_record_state.route_ready ?
        NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    g_record_state.replay_index = 0u;
}

Nav_Route_Record_State_t nav_route_record_get_state(void)
{
    return g_record_state;
}
