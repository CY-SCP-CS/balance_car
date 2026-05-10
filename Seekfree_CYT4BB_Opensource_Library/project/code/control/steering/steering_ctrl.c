#include "steering_ctrl.h"

#include "../../common/utils.h"

float run_yaw_control(const Ctrl_Input_t *fb,
                      PID_Controller_t *pid_yaw,
                      float max_yaw_rate) {
    if (fb->on_bridge) {
        return 0.0f;
    }

    float yaw_rate_target = fb->steering_cmd * max_yaw_rate;
    float steer_diff = pid_calculate(pid_yaw, yaw_rate_target, fb->gyro_yaw_rate);

    return clamp(steer_diff, -pid_yaw->output_limit, pid_yaw->output_limit);
}
