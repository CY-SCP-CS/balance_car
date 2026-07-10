#include "nav_internal.h"
#include "nav_route_record.h"

#include <math.h>
#include <stddef.h>

#include "../../common/utils.h"

#define NAV_DEFAULT_ROUTE_LEN   4u

static const Nav_Segment_t g_default_route[NAV_DEFAULT_ROUTE_LEN] = {
    { NAV_ACTION_GO_STRAIGHT,   3.0f,  0.0f, 0.25f,
      8000u, NAV_LANDMARK_NONE, NAV_REGION_NORMAL },
    { NAV_ACTION_WAIT_LANDMARK, 1.5f,  0.0f, 0.18f,
      5000u, NAV_LANDMARK_WHITE_CIRCLE, NAV_REGION_NORMAL },
    { NAV_ACTION_TURN,          0.0f, 35.0f, 0.0f,
      4000u, NAV_LANDMARK_NONE, NAV_REGION_ROTATE },
    { NAV_ACTION_STOP,          0.0f,  0.0f, 0.0f,
      0u,    NAV_LANDMARK_NONE, NAV_REGION_NONE }
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
static float g_region_start_distance;
static uint32 g_region_start_time;

static float segment_progress(float segment_distance, float target_distance_m)
{
    if (target_distance_m <= 0.0f) {
        return 1.0f;
    }

    return clamp(segment_distance / target_distance_m, 0.0f, 1.0f);
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

static void clear_region_transition(void)
{
    g_state.previous_region = g_state.region;
    g_state.region_entered = false;
    g_state.region_exited = false;
}

static void fill_output_status(Nav_Output_t *out)
{
    if (out == NULL) {
        return;
    }

    out->finished = g_state.finished;
    out->safety_stop = g_state.safety_stop;
    out->region = g_state.region;
    out->region_entered = g_state.region_entered;
    out->region_exited = g_state.region_exited;
}

static void update_region_for_segment(const Nav_Segment_t *seg,
                                      const Nav_Input_t *input)
{
    Nav_Region_t old_region;
    Nav_Region_t new_region;

    if (seg == NULL || input == NULL) {
        return;
    }

    old_region = g_state.region;
    new_region = seg->region;

    if (new_region == old_region) {
        return;
    }

    g_state.previous_region = old_region;
    g_state.region = new_region;
    g_state.region_entered = new_region != NAV_REGION_NONE;
    g_state.region_exited = old_region != NAV_REGION_NONE;
    g_state.region_distance_m = 0.0f;
    g_state.region_elapsed_ms = 0u;

    g_region_start_distance = input->distance_m;
    g_region_start_time = input->time_ms;
}

static void leave_current_region(const Nav_Input_t *input)
{
    if (input == NULL || g_state.region == NAV_REGION_NONE) {
        return;
    }

    g_state.previous_region = g_state.region;
    g_state.region = NAV_REGION_NONE;
    g_state.region_entered = false;
    g_state.region_exited = true;
    g_state.region_distance_m = 0.0f;
    g_state.region_elapsed_ms = 0u;

    g_region_start_distance = input->distance_m;
    g_region_start_time = input->time_ms;
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

    update_region_for_segment(&g_route[index], input);
}

static void advance_segment(const Nav_Input_t *input)
{
    uint8 next = (uint8)(g_state.segment_index + 1u);

    if (next >= g_route_len) {
        leave_current_region(input);
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
    (void)nav_route_record_load_saved();
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
    g_state.region = NAV_REGION_NONE;
    g_state.previous_region = NAV_REGION_NONE;
    g_state.region_entered = false;
    g_state.region_exited = false;
    g_state.region_distance_m = 0.0f;
    g_state.region_elapsed_ms = 0u;
    g_region_start_distance = input->distance_m;
    g_region_start_time = input->time_ms;
    enter_segment(0u, input);
}

void nav_stop(void)
{
    g_state.active = false;
    g_state.finished = false;
    g_state.safety_stop = false;
    g_state.segment_index = 0u;
    g_state.action = NAV_ACTION_IDLE;
    g_state.region = NAV_REGION_NONE;
    g_state.previous_region = NAV_REGION_NONE;
    g_state.region_entered = false;
    g_state.region_exited = false;
    g_state.segment_distance_m = 0.0f;
    g_state.region_distance_m = 0.0f;
    g_state.region_elapsed_ms = 0u;
    g_state.yaw_error_rad = 0.0f;
    g_region_start_distance = 0.0f;
    g_region_start_time = 0u;
}

Nav_Output_t nav_update(const Nav_Input_t *input)
{
    Nav_Output_t out = {
        0.0f, 0.0f, false, false, NAV_REGION_NONE, false, false
    };

    fill_output_status(&out);

    if (input == NULL) {
        return out;
    }

    clear_region_transition();

    if (!g_state.active && !g_state.finished) {
        nav_start(input);
    }

    if (!g_state.active || g_state.finished) {
        fill_output_status(&out);
        nav_route_record_notify_navigation_stopped(out.finished,
                                                   out.safety_stop);
        return out;
    }

    if (input->obstacle_close) {
        g_state.active = false;
        g_state.safety_stop = true;
        fill_output_status(&out);
        nav_route_record_notify_navigation_stopped(out.finished,
                                                   out.safety_stop);
        return out;
    }

    const Nav_Segment_t *seg = &g_route[g_state.segment_index];
    float segment_distance = input->distance_m - g_segment_start_distance;
    float target_yaw_rad = seg->target_yaw_deg * NAV_DEG_TO_RAD;
    float final_yaw = g_segment_start_yaw + target_yaw_rad;
    float yaw_error = nav_wrap_pi(final_yaw - input->yaw_rad);
    bool timed_out = timeout_elapsed(input, seg);

    g_state.segment_distance_m = segment_distance;
    g_state.region_distance_m = input->distance_m - g_region_start_distance;
    g_state.region_elapsed_ms = (uint32)(input->time_ms - g_region_start_time);

    switch (seg->action) {
    case NAV_ACTION_GO_STRAIGHT:
    {
        float progress = segment_progress(segment_distance, seg->target_distance_m);
        float hold_yaw = g_segment_start_yaw + target_yaw_rad * progress;
        yaw_error = nav_wrap_pi(hold_yaw - input->yaw_rad);
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
    }

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
        leave_current_region(input);
        g_state.active = false;
        g_state.finished = true;
        break;
    }

    g_state.yaw_error_rad = yaw_error;
    out.steering_cmd = clamp(out.steering_cmd,
                             -g_cfg.steering_limit,
                             g_cfg.steering_limit);
    fill_output_status(&out);
    nav_route_record_notify_navigation_stopped(out.finished,
                                               out.safety_stop);

    return out;
}

Nav_State_t nav_get_state(void)
{
    return g_state;
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
