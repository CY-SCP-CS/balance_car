#ifndef CONTROL_LEG_CTRL_H
#define CONTROL_LEG_CTRL_H

#include "../../common/types.h"
#include "../kinematics/kinematics.h"
#include "../pid/pid.h"

void run_leg_position_control(float leg_length_delta,
                              const LegConfig_t *leg,
                              const Feedback_Data_t *fb,
                              const Safety_Limits_t *lim,
                              PID_Controller_t *pid_x_L,
                              PID_Controller_t *pid_y_L,
                              PID_Controller_t *pid_x_R,
                              PID_Controller_t *pid_y_R,
                              Output_Data_t *out);

#endif
