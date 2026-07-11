#include "nav_adapter.h"
#include "nav_internal.h"

#include "../robot_control/robot_control.h"

void nav_input_update_from_ctrl(Nav_Input_t *input, const Ctrl_Input_t *ctrl)
{
    static float yaw_zero = 0.0f;
    static bool yaw_zero_set = false;

    if (input == NULL || ctrl == NULL) {
        return;
    }

    if (!yaw_zero_set) {
        yaw_zero = ctrl->body_yaw;
        yaw_zero_set = true;
    }

    input->yaw_rad = nav_wrap_pi(ctrl->body_yaw - yaw_zero);
    input->time_ms += NAV_LOOP_DT_MS;

    input->distance_m = robot_control_get_distance();

    /* Vision updates landmark / obstacle fields after this update. */
}

void nav_apply_ctrl(Ctrl_Input_t *ctrl, const Nav_Output_t *nav)
{
    if (ctrl == NULL || nav == NULL) {
        return;
    }

    ctrl->velocity_cmd = nav->velocity_cmd;
    ctrl->steering_cmd = nav->steering_cmd;
}
