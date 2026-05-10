#ifndef HMI_PAGE_DASHBOARD_H
#define HMI_PAGE_DASHBOARD_H

#include "../../common/types.h"
#include "../../app/navigation/nav_engine.h"

void dashboard_update(const Feedback_Data_t *fb,
                      const Nav_Input_t *nav_input,
                      const Nav_Output_t *nav_output);

#endif
