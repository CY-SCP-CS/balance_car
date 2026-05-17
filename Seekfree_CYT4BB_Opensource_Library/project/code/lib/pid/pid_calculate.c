#include "pid_calculate.h"
#include <math.h>

/*---------------------------------------------------------------------------*/
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd,
              float dt, float out_max, float integral_max){
    pid->kp          = kp;
    pid->ki          = ki;
    pid->kd          = kd;
    pid->dt          = dt;
    pid->out_max     = out_max;
    pid->integral_max = integral_max;

    pid_reset(pid);
}

/*---------------------------------------------------------------------------*/
void pid_reset(PID_Controller_t *pid){
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
}

/*---------------------------------------------------------------------------*/
float pid_calculate(PID_Controller_t *pid, float target, float measure){
    float error = target - measure;

    /* --- Proportional --- */
    float p_out = pid->kp * error;

    /* --- Integral (用上一周期的积分值计算 i_out) --- */
    float i_out = pid->ki * pid->integral;

    /* --- Derivative (on error) + 一阶低通滤波 --- */
    float derivative = (error - pid->prev_error) / pid->dt;
    pid->d_filtered += (derivative - pid->d_filtered) * PID_D_LPF_ALPHA;
    pid->prev_error = error;
    float d_out = pid->kd * pid->d_filtered;

    /* --- Sum and clamp output --- */
    float output = p_out + i_out + d_out;
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < -pid->out_max)
        output = -pid->out_max;

    /* --- 遇限削弱：仅在未饱和或误差有助于退出饱和时积分 --- */
    if (fabsf(output) < pid->out_max || error * pid->integral <= 0) {
        pid->integral += error * pid->dt;
    }
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    return output;
}

/*---------------------------------------------------------------------------*/
float pd_calculate_speed(PID_Controller_t *pid, float target,
                                         float measure, float velocity){
    float error = target - measure;

    float p_out = pid->kp * error;

    float i_out = pid->ki * pid->integral;

    pid->d_filtered += (velocity - pid->d_filtered) * PID_D_LPF_ALPHA;
    float d_out = -pid->kd * pid->d_filtered;

    float output = p_out + i_out + d_out;
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < -pid->out_max)
        output = -pid->out_max;

    /* 遇限削弱 */
    if (fabsf(output) < pid->out_max || error * pid->integral <= 0) {
        pid->integral += error * pid->dt;
    }
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    return output;
}
