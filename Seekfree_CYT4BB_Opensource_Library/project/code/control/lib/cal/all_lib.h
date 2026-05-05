#ifndef ALL_LIB_H
#define ALL_LIB_H

#include <stdbool.h>


float clamp(float val, float min, float max);

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;     // 积分限幅
    float output_limit;      // 输出限幅
    float error;            // 当前误差
    float prev_error;       // 上次误差
    float integral;         // 积分累加值
    float output;           // 当前输出
} PID_Controller_t;


typedef struct {
    float kp;
    float kd;
    float output_limit;
    float output;
} PD_Controller_t;


float pid_calculate(PID_Controller_t *pid, float target, float measure);


float pd_calculate(PD_Controller_t *pd, float target, float measure, float speed);

#endif // ALL_LIB_H
