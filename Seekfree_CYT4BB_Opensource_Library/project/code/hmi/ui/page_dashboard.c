#include "page_dashboard.h"

void page_dashboard_update(const UI_Frame_t *frame)
{
    const Ctrl_Input_t *fb         = frame->fb;
    const Nav_Output_t *nav_output = frame->nav_output;
    Nav_State_t          state     = nav_get_state();

    ui_scope6_send(
        fb != NULL ? fb->body_pitch * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->body_roll * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_roll_rate * UI_RAD_TO_DEG : 0.0f,
        nav_output != NULL ? nav_output->velocity_cmd : 0.0f,
        nav_output != NULL ? nav_output->steering_cmd : 0.0f,
        ui_segment_or_stop(nav_output, state.segment_index));
}
