#include "page_nav_debug.h"

void page_nav_debug_update(const UI_Frame_t *frame)
{
    const Nav_Input_t *nav_input = frame->nav_input;
    const Nav_State_t *state     = frame->nav_state;

    ui_scope6_send(
        nav_input != NULL ? nav_input->distance_m : 0.0f,
        nav_input != NULL ? nav_input->yaw_rad * UI_RAD_TO_DEG : 0.0f,
        state != NULL ? state->segment_distance_m : 0.0f,
        state != NULL ? state->yaw_error_rad * UI_RAD_TO_DEG : 0.0f,
        state != NULL ? (float)state->segment_index : 0.0f,
        state != NULL ? (float)state->action : 0.0f);
}
