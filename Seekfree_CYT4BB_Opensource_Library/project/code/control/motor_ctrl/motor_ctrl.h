#ifndef CONTROL_MOTOR_CTRL_H
#define CONTROL_MOTOR_CTRL_H

#include "../../common/types.h"
#include "../pid/pid.h"

void run_joint_pd_control(const Ctrl_Output_t *target,
                          const Ctrl_Input_t *fb,
                          PD_Controller_t *pd_fl,
                          PD_Controller_t *pd_bl,
                          PD_Controller_t *pd_fr,
                          PD_Controller_t *pd_br,
                          Ctrl_Output_t *out);

void run_torque_distribution(float total_torque,
                             float steer_diff,
                             const Safety_Limits_t *lim,
                             Ctrl_Output_t *out);

#endif
