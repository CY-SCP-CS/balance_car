#include "nav_route_record.h"
#include "nav_internal.h"
#include "nav_route_storage.h"

#include <math.h>
#include <stddef.h>

#include "../../common/utils.h"
#include "../../hmi/indicator/led_buzzer.h"

#define NAV_RECORD_STRAIGHT_SPEED  0.3f
#define NAV_RECORD_CORNER_SPEED    0.18f
#define NAV_RECORD_TURN_SPEED      0.0f
#define NAV_RECORD_WAYPOINT_REACHED_M          0.03f
#define NAV_RECORD_FINAL_BRAKE_SPEED           (-0.2f)
#define NAV_RECORD_FINAL_BRAKE_TIME_MS         260u
#define NAV_RECORD_WAYPOINT_PASS_CROSSTRACK_M  0.04f
#define NAV_RECORD_SHORT_SEGMENT_M             0.015f
#define NAV_RECORD_CROSSTRACK_GAIN             2.0f
#define NAV_RECORD_CROSSTRACK_LIMIT_RAD        (20.0f * NAV_DEG_TO_RAD)
#define NAV_RECORD_SLOW_YAW_ERROR_RAD          (10.0f * NAV_DEG_TO_RAD)
#define NAV_RECORD_TURN_IN_PLACE_YAW_RAD       (25.0f * NAV_DEG_TO_RAD)
#define NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M  0.16f
#define NAV_RECORD_ROTATE_CRAWL_DISTANCE_M     0.07f
#define NAV_RECORD_ROTATE_HARD_BRAKE_DISTANCE_M 0.035f
#define NAV_RECORD_ROTATE_CRAWL_SPEED          0.08f
#define NAV_RECORD_ROTATE_HARD_BRAKE_SPEED     (-0.10f)

static Nav_Keypoint_t g_record_keypoints[NAV_RECORD_MAX_KEYPOINTS];
static Nav_Route_Record_State_t g_record_state;
static float g_record_origin_x;
static float g_record_origin_y;
static float g_record_origin_distance;
static float g_record_origin_yaw;
static float g_record_last_yaw;
static float g_record_yaw_accum;
static uint32 g_record_origin_time;
static float g_replay_start_x;
static float g_replay_start_y;
static float g_replay_start_yaw;
static bool g_replay_final_braking;
static uint32 g_replay_final_brake_start_time;
static float g_replay_final_brake_yaw;

static void record_set_origin(const Nav_Input_t *input)
{
    g_record_origin_x = input->x_m;
    g_record_origin_y = input->y_m;
    g_record_origin_distance = input->distance_m;
    g_record_origin_yaw = input->yaw_rad;
    g_record_last_yaw = input->yaw_rad;
    g_record_yaw_accum = 0.0f;
    g_record_origin_time = input->time_ms;
}

static void record_keypoint_from_input(uint8 index, const Nav_Input_t *input)
{
    float dx = input->x_m - g_record_origin_x;
    float dy = input->y_m - g_record_origin_y;
    float cos_yaw = cosf(g_record_origin_yaw);
    float sin_yaw = sinf(g_record_origin_yaw);
    float relative_distance = input->distance_m - g_record_origin_distance;

    if (index == 0u) {
        g_record_yaw_accum = 0.0f;
    } else {
        g_record_yaw_accum += nav_wrap_pi(input->yaw_rad - g_record_last_yaw);
    }
    g_record_last_yaw = input->yaw_rad;

    g_record_keypoints[index].x_m = cos_yaw * dx + sin_yaw * dy;
    g_record_keypoints[index].y_m = -sin_yaw * dx + cos_yaw * dy;
    g_record_keypoints[index].distance_m =
        relative_distance > 0.0f ? relative_distance : 0.0f;
    g_record_keypoints[index].yaw_rad = g_record_yaw_accum;
    g_record_keypoints[index].time_ms =
        (uint32)(input->time_ms - g_record_origin_time);
    g_record_keypoints[index].action = NAV_ROUTE_POINT_ACTION_NONE;
}

static float keypoint_yaw_delta_from_start(uint8 index)
{
    if (g_record_state.keypoint_count == 0u) {
        return 0.0f;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    return g_record_keypoints[index].yaw_rad;
}

static void replay_transform_keypoint(uint8 index,
                                      float *x_m,
                                      float *y_m,
                                      float *yaw_rad)
{
    const Nav_Keypoint_t *keypoint;
    float cos_yaw;
    float sin_yaw;
    float dx;
    float dy;

    if (g_record_state.keypoint_count == 0u) {
        if (x_m != NULL) {
            *x_m = g_replay_start_x;
        }
        if (y_m != NULL) {
            *y_m = g_replay_start_y;
        }
        if (yaw_rad != NULL) {
            *yaw_rad = g_replay_start_yaw;
        }
        return;
    }

    if (index >= g_record_state.keypoint_count) {
        index = (uint8)(g_record_state.keypoint_count - 1u);
    }

    keypoint = &g_record_keypoints[index];
    cos_yaw = cosf(g_replay_start_yaw);
    sin_yaw = sinf(g_replay_start_yaw);
    dx = keypoint->x_m;
    dy = keypoint->y_m;

    if (x_m != NULL) {
        *x_m = g_replay_start_x + cos_yaw * dx - sin_yaw * dy;
    }
    if (y_m != NULL) {
        *y_m = g_replay_start_y + sin_yaw * dx + cos_yaw * dy;
    }
    if (yaw_rad != NULL) {
        *yaw_rad = nav_wrap_pi(g_replay_start_yaw +
                               keypoint_yaw_delta_from_start(index));
    }
}

static bool waypoint_passed(float prev_x,
                            float prev_y,
                            float target_x,
                            float target_y,
                            float current_x,
                            float current_y)
{
    float vx = target_x - prev_x;
    float vy = target_y - prev_y;
    float wx = current_x - prev_x;
    float wy = current_y - prev_y;
    float segment_len_sq = vx * vx + vy * vy;
    float cross_track;

    if (segment_len_sq <= 0.000001f) {
        return false;
    }

    if ((vx * wx + vy * wy) < segment_len_sq) {
        return false;
    }

    cross_track = fabsf(vx * wy - vy * wx) / sqrtf(segment_len_sq);
    return cross_track <= NAV_RECORD_WAYPOINT_PASS_CROSSTRACK_M;
}

static void replay_advance_waypoint(void)
{
    g_record_state.replay_index++;
    buzzer_beep(BEEP_SHORT);
}

static void replay_fill_waypoint_event(Nav_Output_t *out, uint8 index)
{
    if (out == NULL || index >= g_record_state.keypoint_count) {
        return;
    }

    out->waypoint_entered = true;
    out->waypoint_index = index;
    out->waypoint_action = g_record_keypoints[index].action;
}

static void replay_reset_final_brake(void)
{
    g_replay_final_braking = false;
    g_replay_final_brake_start_time = 0u;
    g_replay_final_brake_yaw = 0.0f;
}

static void replay_hold_current_yaw(Nav_Output_t *out, const Nav_Input_t *input)
{
    if (out == NULL || input == NULL) {
        return;
    }

    out->velocity_cmd = 0.0f;
    out->target_yaw_valid = true;
    out->target_yaw_rad = input->yaw_rad;
}

static void replay_apply_rotate_prebrake(Nav_Output_t *out,
                                         uint8 target_index,
                                         float target_distance)
{
    float ratio;
    float speed_range;

    if (out == NULL || target_index >= g_record_state.keypoint_count ||
        g_record_keypoints[target_index].action !=
        NAV_ROUTE_POINT_ACTION_ROTATE720) {
        return;
    }

    if (target_distance <= NAV_RECORD_ROTATE_HARD_BRAKE_DISTANCE_M) {
        out->velocity_cmd = NAV_RECORD_ROTATE_HARD_BRAKE_SPEED;
        return;
    }

    if (target_distance <= NAV_RECORD_ROTATE_CRAWL_DISTANCE_M) {
        out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED;
        return;
    }

    if (target_distance >= NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M) {
        return;
    }

    speed_range = NAV_RECORD_ROTATE_PREBRAKE_DISTANCE_M -
                  NAV_RECORD_ROTATE_CRAWL_DISTANCE_M;
    if (speed_range <= 0.0f) {
        out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED;
        return;
    }

    ratio = (target_distance - NAV_RECORD_ROTATE_CRAWL_DISTANCE_M) /
            speed_range;
    ratio = clamp(ratio, 0.0f, 1.0f);
    out->velocity_cmd = NAV_RECORD_ROTATE_CRAWL_SPEED +
        (out->velocity_cmd - NAV_RECORD_ROTATE_CRAWL_SPEED) * ratio;
}

static Nav_Output_t replay_final_brake_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {0};
    uint32 elapsed_ms;

    if (input == NULL) {
        return out;
    }

    elapsed_ms = input->time_ms - g_replay_final_brake_start_time;
    if (elapsed_ms >= NAV_RECORD_FINAL_BRAKE_TIME_MS) {
        replay_reset_final_brake();
        g_record_state.mode = NAV_ROUTE_READY;
        g_record_state.replay_index = 0u;
        out.finished = true;
        replay_hold_current_yaw(&out, input);
        return out;
    }

    out.velocity_cmd = NAV_RECORD_FINAL_BRAKE_SPEED;
    out.target_yaw_valid = true;
    out.target_yaw_rad = g_replay_final_brake_yaw;
    out.region = NAV_REGION_NORMAL;
    return out;
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
    replay_reset_final_brake();

    record_set_origin(input);
    record_keypoint_from_input(0u, input);

    return true;
}

bool nav_route_record_keypoint(const Nav_Input_t *input)
{
    return nav_route_record_keypoint_with_action(input,
                                                 NAV_ROUTE_POINT_ACTION_NONE);
}

bool nav_route_record_keypoint_with_action(const Nav_Input_t *input,
                                           Nav_Route_Point_Action_t action)
{
    if (input == NULL || g_record_state.mode != NAV_ROUTE_RECORDING) {
        return false;
    }

    if (g_record_state.keypoint_count >= NAV_RECORD_MAX_KEYPOINTS) {
        g_record_state.overflow = true;
        return false;
    }

    uint8 index = g_record_state.keypoint_count;
    record_keypoint_from_input(index, input);
    g_record_keypoints[index].action = action;
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
    replay_reset_final_brake();
}

static bool set_loaded_keypoints_ready(uint8 keypoint_count)
{
    nav_stop();
    g_record_state.mode = NAV_ROUTE_IDLE;
    g_record_state.keypoint_count = keypoint_count;
    g_record_state.replay_index = 0u;
    g_record_state.route_ready = false;
    g_record_state.overflow = false;
    replay_reset_final_brake();

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
    g_replay_start_x = input->x_m;
    g_replay_start_y = input->y_m;
    g_replay_start_yaw = input->yaw_rad;
    replay_reset_final_brake();

    return true;
}

Nav_Output_t nav_route_replay_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {0};
    Nav_Config_t cfg;
    float reach_radius;

    if (input == NULL ||
        g_record_state.mode != NAV_ROUTE_REPLAYING ||
        !g_record_state.route_ready ||
        g_record_state.keypoint_count < 2u) {
        return out;
    }

    if (input->obstacle_close) {
        g_record_state.mode = NAV_ROUTE_READY;
        g_record_state.replay_index = 0u;
        replay_reset_final_brake();
        out.safety_stop = true;
        replay_hold_current_yaw(&out, input);
        return out;
    }

    if (g_replay_final_braking) {
        return replay_final_brake_update(input);
    }

    cfg = nav_get_config();
    reach_radius = cfg.distance_tolerance_m;
    if (reach_radius <= 0.0f ||
        reach_radius > NAV_RECORD_WAYPOINT_REACHED_M) {
        reach_radius = NAV_RECORD_WAYPOINT_REACHED_M;
    }

    while (g_record_state.replay_index < g_record_state.keypoint_count) {
        uint8 prev_index = (uint8)(g_record_state.replay_index - 1u);
        uint8 cur_index = g_record_state.replay_index;
        float prev_x;
        float prev_y;
        float target_x;
        float target_y;
        float target_yaw_cmd;
        float segment_dx;
        float segment_dy;
        float segment_distance;
        float target_dx;
        float target_dy;
        float target_distance;
        float segment_yaw;
        float cross_track_error;
        float yaw_correction;
        float yaw_error;
        float abs_yaw_error;
        bool passed_waypoint;
        bool final_waypoint;

        replay_transform_keypoint(prev_index, &prev_x, &prev_y, NULL);
        replay_transform_keypoint(cur_index, &target_x, &target_y, NULL);
        final_waypoint = (cur_index + 1u >= g_record_state.keypoint_count);

        segment_dx = target_x - prev_x;
        segment_dy = target_y - prev_y;
        segment_distance = sqrtf(segment_dx * segment_dx +
                                 segment_dy * segment_dy);
        segment_yaw = atan2f(segment_dy, segment_dx);

        if (segment_distance < NAV_RECORD_SHORT_SEGMENT_M) {
            replay_fill_waypoint_event(&out, cur_index);
            replay_advance_waypoint();
            if (out.waypoint_action != NAV_ROUTE_POINT_ACTION_NONE) {
                replay_hold_current_yaw(&out, input);
                return out;
            }
            continue;
        }

        target_dx = target_x - input->x_m;
        target_dy = target_y - input->y_m;
        target_distance = sqrtf(target_dx * target_dx +
                                target_dy * target_dy);
        passed_waypoint = waypoint_passed(prev_x,
                                          prev_y,
                                          target_x,
                                          target_y,
                                          input->x_m,
                                          input->y_m);

        if (final_waypoint && passed_waypoint) {
            Nav_Output_t brake_out;
            replay_advance_waypoint();
            g_replay_final_braking = true;
            g_replay_final_brake_start_time = input->time_ms;
            g_replay_final_brake_yaw = segment_yaw;
            brake_out = replay_final_brake_update(input);
            replay_fill_waypoint_event(&brake_out, cur_index);
            return brake_out;
        }

        if (!final_waypoint &&
            (target_distance <= reach_radius || passed_waypoint)) {
            replay_fill_waypoint_event(&out, cur_index);
            replay_advance_waypoint();
            replay_hold_current_yaw(&out, input);
            return out;
        }

        cross_track_error =
            -sinf(segment_yaw) * (input->x_m - prev_x) +
             cosf(segment_yaw) * (input->y_m - prev_y);
        yaw_correction = clamp(-NAV_RECORD_CROSSTRACK_GAIN * cross_track_error,
                               -NAV_RECORD_CROSSTRACK_LIMIT_RAD,
                               NAV_RECORD_CROSSTRACK_LIMIT_RAD);
        target_yaw_cmd = nav_wrap_pi(segment_yaw + yaw_correction);
        yaw_error = nav_wrap_pi(target_yaw_cmd - input->yaw_rad);
        abs_yaw_error = fabsf(yaw_error);

        if (abs_yaw_error >= NAV_RECORD_TURN_IN_PLACE_YAW_RAD) {
            out.velocity_cmd = NAV_RECORD_TURN_SPEED;
        } else {
            out.velocity_cmd = NAV_RECORD_STRAIGHT_SPEED;
            if (abs_yaw_error >= NAV_RECORD_SLOW_YAW_ERROR_RAD) {
                out.velocity_cmd = NAV_RECORD_CORNER_SPEED;
            }
        }
        replay_apply_rotate_prebrake(&out, cur_index, target_distance);
        out.target_yaw_valid = true;
        out.target_yaw_rad = target_yaw_cmd;
        out.region = NAV_REGION_NORMAL;
        return out;
    }

    g_record_state.mode = NAV_ROUTE_READY;
    g_record_state.replay_index = 0u;
    replay_reset_final_brake();
    out.finished = true;
    replay_hold_current_yaw(&out, input);
    return out;
}

void nav_route_replay_stop(void)
{
    nav_stop();
    g_record_state.mode = g_record_state.route_ready ?
        NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    g_record_state.replay_index = 0u;
    replay_reset_final_brake();
}

Nav_Route_Record_State_t nav_route_record_get_state(void)
{
    return g_record_state;
}
