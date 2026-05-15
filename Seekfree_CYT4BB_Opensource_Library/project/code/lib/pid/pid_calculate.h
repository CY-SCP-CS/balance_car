#ifndef PID_CALCULATE_H
#define PID_CALCULATE_H

#include <stdint.h>

/* 微分项一阶低通滤波系数 (0~1), 越大滤波越弱 */
#define PID_D_LPF_ALPHA     0.15f

typedef struct
{
    float kp;
    float ki;
    float kd;
    float dt;
    float integral;
    float prev_error;
    float d_filtered;   /* 低通滤波后的微分值 */
    float out_max;
    float integral_max;
} PID_Controller_t;

void pid_init(PID_Controller_t *pid, float kp, float ki, float kd,
              float dt, float out_max, float integral_max);
void pid_reset(PID_Controller_t *pid);
float pid_calculate(PID_Controller_t *pid, float target, float measure);
float pd_calculate_speed(PID_Controller_t *pid, float target,
                                         float measure, float velocity);

#endif /* PID_CALCULATE_H */
