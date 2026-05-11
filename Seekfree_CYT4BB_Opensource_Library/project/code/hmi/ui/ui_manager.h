#ifndef HMI_UI_MANAGER_H
#define HMI_UI_MANAGER_H

#include "../../common/types.h"
#include "../../app/navigation/nav_engine.h"
#include "../../app/vision/vision_pipeline.h"

typedef enum {
    UI_PAGE_DASHBOARD,
    UI_PAGE_IMU_DEBUG,
    UI_PAGE_NAV_DEBUG,
    UI_PAGE_PID_DEBUG,
    UI_PAGE_REMOTE,
} UI_Page_t;

void ui_init(UI_Page_t page);
void ui_update(const Ctrl_Input_t    *fb,
               const Nav_Input_t     *nav_input,
               const Nav_Output_t    *nav_output,
               const Vision_Result_t *vision);

#endif
