#ifndef CONTROL_STEERING_CTRL_H
#define CONTROL_STEERING_CTRL_H

#include "../../common/types.h"
#include "../pid/pid.h"

float run_yaw_control(const Feedback_Data_t *fb,
                      PID_Controller_t *pid_yaw,
                      float max_yaw_rate);

#endif
