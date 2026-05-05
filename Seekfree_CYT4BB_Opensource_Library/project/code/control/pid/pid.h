#ifndef CONTROL_PID_H
#define CONTROL_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float error;
    float prev_error;
    float integral;
    float output;
} PID_Controller_t;

typedef struct {
    float kp;
    float kd;
    float output_limit;
    float output;
} PD_Controller_t;

float pid_calculate(PID_Controller_t *pid, float target, float measure);
float pd_calculate(PD_Controller_t *pd, float target, float measure, float speed);

#endif
