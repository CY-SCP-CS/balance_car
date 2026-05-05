#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>


typedef struct {
    float l1;                       // 大腿长度 (mm)
    float l2;                       // 小腿长度 (mm)
    float hip_offset;               // 左右电机轴心间距的一半 (mm)
    float nominal_leg_length;       // 标准腿长 (mm)
    float wheel_radius;             // 车轮半径 (mm)
} LegConfig_t;


typedef struct {
    float x;
    float y;
} Foot_Position_t;


typedef struct {
    bool valid;                      // 解是否有效
    Foot_Position_t left;           // 左腿足端
    Foot_Position_t right;          // 右腿足端
} Kinematics_State_t;

//正解算
void five_bar_fk(const LegConfig_t *leg, float theta_f, float theta_b, float *x, float *y);

//逆解算
void five_bar_ik(const LegConfig_t *leg, float x, float y, float *theta_f, float *theta_b);

//当前状态
Kinematics_State_t calc_kinematics_state(const LegConfig_t *leg,
                                          float theta_f_L, float theta_b_L,
                                          float theta_f_R, float theta_b_R);

#endif 
