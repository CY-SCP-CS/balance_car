#include "page_nav.h"

#include "seekfree_assistant.h"

#define NAV_RAD_TO_DEG     (180.0f / 3.14159265f)

void nav_debug_display(const Nav_Input_t *input)
{
    Nav_State_t state = nav_get_state();

    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] = input != 0 ? input->distance_m : 0.0f;
    seekfree_assistant_oscilloscope_data.data[1] = input != 0 ? input->yaw_rad * NAV_RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[2] = state.segment_distance_m;
    seekfree_assistant_oscilloscope_data.data[3] = state.yaw_error_rad * NAV_RAD_TO_DEG;
    seekfree_assistant_oscilloscope_data.data[4] = (float)state.segment_index;
    seekfree_assistant_oscilloscope_data.data[5] = (float)state.action;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
