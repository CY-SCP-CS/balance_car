#include "nav_engine.h"

#include <math.h>
#include <stddef.h>

#include "../../common/utils.h"
#include "../robot_control/robot_control.h"

#define NAV_PI                  3.14159265f
#define NAV_DEG_TO_RAD          (NAV_PI / 180.0f)
#define NAV_RAD_TO_DEG          (180.0f / NAV_PI)
#define NAV_DEFAULT_ROUTE_LEN   4u
#define NAV_RECORD_STRAIGHT_SPEED  0.18f
#define NAV_RECORD_TURN_SPEED      0.0f
#define NAV_RECORD_MIN_DISTANCE_M  0.06f
#define NAV_RECORD_MIN_TURN_RAD    (12.0f * NAV_DEG_TO_RAD)
#define NAV_RECORD_STRAIGHT_TIMEOUT_BASE_MS  2000u
#define NAV_RECORD_STRAIGHT_TIMEOUT_PER_M_MS 7000u
#define NAV_RECORD_TURN_TIMEOUT_BASE_MS      1500u
#define NAV_RECORD_TURN_TIMEOUT_PER_DEG_MS   35u
#define NAV_RECORD_TIMEOUT_MAX_MS            15000u

static const Nav_Segment_t g_default_route[NAV_DEFAULT_ROUTE_LEN] = {
    { NAV_ACTION_GO_STRAIGHT,   3.0f,  0.0f, 0.25f, 8000u, NAV_LANDMARK_NONE },
    { NAV_ACTION_WAIT_LANDMARK, 1.5f,  0.0f, 0.18f, 5000u, NAV_LANDMARK_WHITE_CIRCLE },
    { NAV_ACTION_TURN,          0.0f, 35.0f, 0.0f,  4000u, NAV_LANDMARK_NONE },
    { NAV_ACTION_STOP,          0.0f,  0.0f, 0.0f,     0u, NAV_LANDMARK_NONE }
};

static Nav_Config_t g_cfg = {
    1.2f,
    1.8f,
    0.35f,
    1.0f,
    0.08f,
    4.0f * NAV_DEG_TO_RAD,
    3u
};

static const Nav_Segment_t *g_route = g_default_route;
static uint8 g_route_len = NAV_DEFAULT_ROUTE_LEN;
static Nav_State_t g_state;
static float g_segment_start_distance;
static float g_segment_start_yaw;
static uint32 g_segment_start_time;
static Nav_Keypoint_t g_record_keypoints[NAV_RECORD_MAX_KEYPOINTS];
static Nav_Segment_t g_record_route[NAV_RECORD_MAX_SEGMENTS];
static Nav_Route_Record_State_t g_record_state;

static float wrap_pi(float angle)
{
    while (angle > NAV_PI) {
        angle -= 2.0f * NAV_PI;
    }

    while (angle < -NAV_PI) {
        angle += 2.0f * NAV_PI;
    }

    return angle;
}

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
        float delta_yaw = wrap_pi(cur->yaw_rad - prev->yaw_rad);

        if (delta_distance < 0.0f) {
            delta_distance = 0.0f;
        }

        if (delta_distance >= NAV_RECORD_MIN_DISTANCE_M) {
            uint32 timeout_ms = NAV_RECORD_STRAIGHT_TIMEOUT_BASE_MS +
                (uint32)(delta_distance * (float)NAV_RECORD_STRAIGHT_TIMEOUT_PER_M_MS);

            if (!append_record_segment(NAV_ACTION_GO_STRAIGHT,
                                       delta_distance,
                                       0.0f,
                                       NAV_RECORD_STRAIGHT_SPEED,
                                       limit_timeout(timeout_ms))) {
                return false;
            }
        }

        if (fabsf(delta_yaw) >= NAV_RECORD_MIN_TURN_RAD) {
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

static bool timeout_elapsed(const Nav_Input_t *input, const Nav_Segment_t *seg)
{
    if (seg->timeout_ms == 0u) {
        return false;
    }

    return (uint32)(input->time_ms - g_segment_start_time) >= seg->timeout_ms;
}

static bool landmark_seen(const Nav_Input_t *input, Nav_Landmark_t landmark)
{
    return landmark != NAV_LANDMARK_NONE &&
           input->landmark == landmark &&
           input->landmark_confidence >= g_cfg.landmark_min_confidence;
}

static void enter_segment(uint8 index, const Nav_Input_t *input)
{
    g_state.segment_index = index;
    g_state.action = g_route[index].action;
    g_state.segment_distance_m = 0.0f;
    g_state.yaw_error_rad = 0.0f;

    g_segment_start_distance = input->distance_m;
    g_segment_start_yaw = input->yaw_rad;
    g_segment_start_time = input->time_ms;
}

static void advance_segment(const Nav_Input_t *input)
{
    uint8 next = (uint8)(g_state.segment_index + 1u);

    if (next >= g_route_len) {
        g_state.active = false;
        g_state.finished = true;
        g_state.action = NAV_ACTION_STOP;
        return;
    }

    enter_segment(next, input);
}

void nav_init(const Nav_Config_t *config)
{
    if (config != NULL) {
        g_cfg = *config;
    }

    g_route = g_default_route;
    g_route_len = NAV_DEFAULT_ROUTE_LEN;
    nav_stop();
    nav_route_record_reset();
}

void nav_set_route(const Nav_Segment_t *route, uint8 route_len)
{
    if (route == NULL || route_len == 0u) {
        g_route = g_default_route;
        g_route_len = NAV_DEFAULT_ROUTE_LEN;
        return;
    }

    g_route = route;
    g_route_len = route_len;
}

void nav_start(const Nav_Input_t *input)
{
    if (input == NULL || g_route_len == 0u) {
        nav_stop();
        return;
    }

    g_state.active = true;
    g_state.finished = false;
    g_state.safety_stop = false;
    enter_segment(0u, input);
}

void nav_stop(void)
{
    g_state.active = false;
    g_state.finished = false;
    g_state.safety_stop = false;
    g_state.segment_index = 0u;
    g_state.action = NAV_ACTION_IDLE;
    g_state.segment_distance_m = 0.0f;
    g_state.yaw_error_rad = 0.0f;
}

Nav_Output_t nav_update(const Nav_Input_t *input)
{
    Nav_Output_t out = { 0.0f, 0.0f, g_state.finished, g_state.safety_stop };

    if (input == NULL) {
        return out;
    }

    if (!g_state.active && !g_state.finished) {
        nav_start(input);
    }

    if (!g_state.active || g_state.finished) {
        out.finished = g_state.finished;
        if (g_record_state.mode == NAV_ROUTE_REPLAYING &&
            (out.finished || out.safety_stop)) {
            g_record_state.mode = g_record_state.route_ready ?
                NAV_ROUTE_READY : NAV_ROUTE_IDLE;
        }
        return out;
    }

    if (input->obstacle_close) {
        g_state.active = false;
        g_state.safety_stop = true;
        out.safety_stop = true;
        if (g_record_state.mode == NAV_ROUTE_REPLAYING) {
            g_record_state.mode = g_record_state.route_ready ?
                NAV_ROUTE_READY : NAV_ROUTE_IDLE;
        }
        return out;
    }

    const Nav_Segment_t *seg = &g_route[g_state.segment_index];
    float segment_distance = input->distance_m - g_segment_start_distance;
    float hold_yaw = g_segment_start_yaw + seg->target_yaw_deg * NAV_DEG_TO_RAD;
    float yaw_error = wrap_pi(hold_yaw - input->yaw_rad);
    bool timed_out = timeout_elapsed(input, seg);

    g_state.segment_distance_m = segment_distance;
    g_state.yaw_error_rad = yaw_error;

    switch (seg->action) {
    case NAV_ACTION_GO_STRAIGHT:
        out.velocity_cmd = seg->target_speed;
        out.steering_cmd = g_cfg.yaw_kp * yaw_error;

        if (landmark_seen(input, seg->landmark)) {
            out.steering_cmd += g_cfg.landmark_kp * input->landmark_offset;
        }

        if (segment_distance >= seg->target_distance_m - g_cfg.distance_tolerance_m ||
            timed_out) {
            advance_segment(input);
        }
        break;

    case NAV_ACTION_WAIT_LANDMARK:
        out.velocity_cmd = seg->target_speed;
        out.steering_cmd = g_cfg.yaw_kp * yaw_error;

        if (landmark_seen(input, seg->landmark)) {
            out.steering_cmd += g_cfg.landmark_kp * input->landmark_offset;
            advance_segment(input);
        } else if ((seg->target_distance_m > 0.0f &&
                    segment_distance >= seg->target_distance_m - g_cfg.distance_tolerance_m) ||
                   timed_out) {
            advance_segment(input);
        }
        break;

    case NAV_ACTION_TURN:
        out.velocity_cmd = seg->target_speed;
        out.steering_cmd = g_cfg.turn_kp * yaw_error;

        if (fabsf(yaw_error) <= g_cfg.yaw_tolerance_rad || timed_out) {
            advance_segment(input);
        }
        break;

    case NAV_ACTION_STOP:
    default:
        g_state.active = false;
        g_state.finished = true;
        break;
    }

    out.steering_cmd = clamp(out.steering_cmd,
                             -g_cfg.steering_limit,
                             g_cfg.steering_limit);
    out.finished = g_state.finished;
    out.safety_stop = g_state.safety_stop;
    if (g_record_state.mode == NAV_ROUTE_REPLAYING &&
        (out.finished || out.safety_stop)) {
        g_record_state.mode = g_record_state.route_ready ?
            NAV_ROUTE_READY : NAV_ROUTE_IDLE;
    }

    return out;
}

Nav_State_t nav_get_state(void)
{
    return g_state;
}

void nav_input_update_from_ctrl(Nav_Input_t *input, const Ctrl_Input_t *ctrl)
{
    static float yaw_rad = 0.0f;

    if (input == NULL || ctrl == NULL) {
        return;
    }

    yaw_rad = wrap_pi(yaw_rad + ctrl->gyro_yaw_rate * NAV_LOOP_DT_S);

    input->yaw_rad = yaw_rad;
    input->time_ms += NAV_LOOP_DT_MS;

    input->distance_m = robot_control_get_distance();

    /* landmark / obstacle fields are populated by vision_feed_nav_input()
       which the caller must invoke before calling nav_input_update_from_ctrl(). */
}

void nav_apply_ctrl(Ctrl_Input_t *ctrl, const Nav_Output_t *nav)
{
    if (ctrl == NULL || nav == NULL) {
        return;
    }

    ctrl->velocity_cmd = nav->velocity_cmd;
    ctrl->steering_cmd = nav->steering_cmd;
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

Nav_Config_t nav_get_config(void)
{
    return g_cfg;
}

void nav_set_config(const Nav_Config_t *config)
{
    if (config == NULL) {
        return;
    }

    g_cfg = *config;
}

Nav_Config_t *nav_config(void)
{
    return &g_cfg;
}
