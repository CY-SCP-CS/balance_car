#include "nav_internal.h"

#include "../robot_control/robot_control.h"

void nav_input_update_from_ctrl(Nav_Input_t *input, const Ctrl_Input_t *ctrl)
{
    static float yaw_rad = 0.0f;

    if (input == NULL || ctrl == NULL) {
        return;
    }

    yaw_rad = nav_wrap_pi(yaw_rad + ctrl->gyro_yaw_rate * NAV_LOOP_DT_S);

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
