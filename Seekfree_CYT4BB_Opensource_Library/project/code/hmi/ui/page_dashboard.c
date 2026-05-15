#include "page_dashboard.h"

#include "seekfree_assistant.h"

#define DASHBOARD_RAD_TO_DEG    (180.0f / 3.14159265f)

void dashboard_update(const Ctrl_Input_t *fb,
                      const Nav_Input_t *nav_input,
                      const Nav_Output_t *nav_output)
{
    Nav_State_t state = nav_get_state();

    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] =
        fb != 0 ? fb->body_pitch * DASHBOARD_RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[1] =
        fb != 0 ? fb->body_roll * DASHBOARD_RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[2] =
        fb != 0 ? fb->gyro_roll_rate * DASHBOARD_RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[3] =
        nav_output != 0 ? nav_output->velocity_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[4] =
        nav_output != 0 ? nav_output->steering_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[5] =
        (nav_output != 0 && nav_output->safety_stop) ? -1.0f : (float)state.segment_index;

    (void)nav_input;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
