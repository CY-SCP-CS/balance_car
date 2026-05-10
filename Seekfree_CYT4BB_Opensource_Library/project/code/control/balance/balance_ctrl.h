#ifndef CONTROL_BALANCE_CTRL_H
#define CONTROL_BALANCE_CTRL_H

#include "../../common/types.h"
#include "../pid/pid.h"

float run_pitch_control(const Ctrl_Input_t *fb,
                        const Safety_Limits_t *lim,
                        PID_Controller_t *outer_pid,
                        PID_Controller_t *inner_pid);

float run_roll_control(const Ctrl_Input_t *fb,
                       const Safety_Limits_t *lim,
                       PID_Controller_t *pid_roll);

#endif
