#include "pid.h"

#include <math.h>

#include "../../common/utils.h"

float pid_calculate(PID_Controller_t *pid, float target, float measure) {
    const float anti_windup_threshold = pid->output_limit;

    pid->error = target - measure;

    if (fabsf(pid->error) < anti_windup_threshold) {
        pid->integral += pid->error;
        pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    } else {
        pid->integral = 0.0f;
    }

    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * (pid->error - pid->prev_error);
    pid->output = clamp(pid->output, -pid->output_limit, pid->output_limit);

    pid->prev_error = pid->error;
    return pid->output;
}

float pd_calculate(PD_Controller_t *pd, float target, float measure, float speed) {
    float error = target - measure;
    float output = pd->kp * error - pd->kd * speed;

    output = clamp(output, -pd->output_limit, pd->output_limit);
    pd->output = output;
    return output;
}
