#include "page_nav_debug.h"

void page_nav_debug_update(const UI_Frame_t *frame)
{
    const Nav_Input_t *nav_input = frame->nav_input;
    Nav_State_t         state    = nav_get_state();

    ui_scope6_send(
        nav_input != NULL ? nav_input->distance_m : 0.0f,
        nav_input != NULL ? nav_input->yaw_rad * UI_RAD_TO_DEG : 0.0f,
        state.segment_distance_m,
        state.yaw_error_rad * UI_RAD_TO_DEG,
        (float)state.segment_index,
        (float)state.action);
}
