#ifndef CONTROL_KINEMATICS_H
#define CONTROL_KINEMATICS_H

#include <stdbool.h>

typedef struct {
    float l1;
    float l2;
    float hip_offset;
    float nominal_leg_length;
    float wheel_radius;
} LegConfig_t;

typedef struct {
    float x;
    float y;
} Foot_Position_t;

typedef struct {
    bool valid;
    Foot_Position_t left;
    Foot_Position_t right;
} Kinematics_State_t;

void five_bar_fk(const LegConfig_t *leg, float theta_f, float theta_b, float *x, float *y);
void five_bar_ik(const LegConfig_t *leg, float x, float y, float *theta_f, float *theta_b);

Kinematics_State_t calc_kinematics_state(const LegConfig_t *leg,
                                         float theta_f_L, float theta_b_L,
                                         float theta_f_R, float theta_b_R);

#endif
