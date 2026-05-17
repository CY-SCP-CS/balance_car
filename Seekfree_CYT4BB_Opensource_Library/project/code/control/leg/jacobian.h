#ifndef JACOBIAN_H
#define JACOBIAN_H

#include "kinematics.h"

int five_bar_jacobian(const Joint_angle_t *angles, float J[2][2]);
#endif /* JACOBIAN_H */
