#ifndef APP_NAVIGATION_NAV_ADAPTER_H
#define APP_NAVIGATION_NAV_ADAPTER_H

#include "nav_engine.h"

#define NAV_LOOP_DT_S   0.001f
#define NAV_LOOP_DT_MS  1u

void nav_input_update_from_ctrl(Nav_Input_t *input, const Ctrl_Input_t *ctrl);
void nav_apply_ctrl(Ctrl_Input_t *ctrl, const Nav_Output_t *nav);

#endif
