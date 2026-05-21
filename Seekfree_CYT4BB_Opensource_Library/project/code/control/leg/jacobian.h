#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "kinematics.h"

int five_bar_jacobian(const Joint_angle_t *angles, float J[2][2]);

/* 求解 J * dθ = [dx; dy] → dθ = J⁻¹ * dfoot, 返回 0=成功 */
int five_bar_jacobian_solve(const float J[2][2],
                            float *dtheta1, float *dtheta2,
                            float dx, float dy);
#endif /* JACOBIAN_H */
