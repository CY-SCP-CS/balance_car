#include "kinematics.h"

#include <math.h>
#include <string.h>

#include "../../common/utils.h"

void five_bar_fk(const LegConfig_t *leg, float theta_f, float theta_b, float *x, float *y) {
    float x_kf = leg->hip_offset + leg->l1 * sinf(theta_f);
    float y_kf = leg->l1 * cosf(theta_f);
    float x_kb = -leg->hip_offset + leg->l1 * sinf(theta_b);
    float y_kb = leg->l1 * cosf(theta_b);

    float dx = x_kb - x_kf;
    float dy = y_kb - y_kf;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 1.0e-6f) {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }

    float half_dist = dist / 2.0f;
    float height = sqrtf(fmaxf(0.0f, leg->l2 * leg->l2 - half_dist * half_dist));
    float x_mid = x_kf + dx / 2.0f;
    float y_mid = y_kf + dy / 2.0f;

    *x = x_mid + height * (dy / dist);
    *y = y_mid - height * (dx / dist);
}

void five_bar_ik(const LegConfig_t *leg, float x, float y, float *theta_f, float *theta_b) {
    float x_df = x - leg->hip_offset;
    float L_f_sq = x_df * x_df + y * y;
    float L_f = sqrtf(L_f_sq);

    if (L_f < 1.0e-6f) {
        L_f = 1.0e-6f;
    }

    float cos_af = clamp((leg->l1 * leg->l1 + L_f_sq - leg->l2 * leg->l2) /
                         (2.0f * leg->l1 * L_f), -1.0f, 1.0f);
    *theta_f = atan2f(x_df, y) - acosf(cos_af);

    float x_db = x + leg->hip_offset;
    float L_b_sq = x_db * x_db + y * y;
    float L_b = sqrtf(L_b_sq);

    if (L_b < 1.0e-6f) {
        L_b = 1.0e-6f;
    }

    float cos_ab = clamp((leg->l1 * leg->l1 + L_b_sq - leg->l2 * leg->l2) /
                         (2.0f * leg->l1 * L_b), -1.0f, 1.0f);
    *theta_b = atan2f(x_db, y) + acosf(cos_ab);
}

Kinematics_State_t calc_kinematics_state(const LegConfig_t *leg,
                                         float theta_f_L, float theta_b_L,
                                         float theta_f_R, float theta_b_R) {
    Kinematics_State_t state;
    memset(&state, 0, sizeof(state));

    five_bar_fk(leg, theta_f_L, theta_b_L, &state.left.x, &state.left.y);
    five_bar_fk(leg, theta_f_R, theta_b_R, &state.right.x, &state.right.y);

    float y_min = leg->nominal_leg_length - leg->l2;
    float y_max = leg->nominal_leg_length + leg->l2;
    state.valid = state.left.y >= y_min && state.left.y <= y_max &&
                  state.right.y >= y_min && state.right.y <= y_max;

    return state;
}
