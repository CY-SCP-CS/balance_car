#include <math.h>
#include "math_utils.h"

// ==============================================
// 限幅工具
// ==============================================
float clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// ==============================================
// PID计算（带抗积分饱和）
// ==============================================
float pid_calculate(PID_Controller_t *pid, float target, float actual, float anti_windup_threshold) {
    pid->error = target - actual;

    // 抗积分饱和：误差小于阈值时才累加积分
    if (fabsf(pid->error) < anti_windup_threshold) {
        pid->integral += pid->error;
        pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    } else {
        pid->integral = 0.0f;
    }

    // PID三项求和
    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * (pid->error - pid->prev_error);

    // 输出限幅
    pid->output = clamp(pid->output, -pid->output_limit, pid->output_limit);

    pid->prev_error = pid->error;
    return pid->output;
}

// ==============================================
// PD计算（用于关节角度闭环）
// ==============================================
float pd_calculate(PD_Controller_t *pd, float target, float actual, float actual_derivative) {
    float error = target - actual;
    float output = pd->kp * error - pd->kd * actual_derivative;
    output = clamp(output, -pd->output_limit, pd->output_limit);
    pd->output = output;
    return output;
}
