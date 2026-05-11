#include "ui_manager.h"

#include "page_dashboard.h"
#include "page_imu_debug.h"
#include "page_nav_debug.h"
#include "page_remote.h"

#include "seekfree_assistant.h"

static UI_Page_t g_page;

void ui_init(UI_Page_t page)
{
    g_page = page;

    if (page == UI_PAGE_REMOTE) {
        remote_page_init();
    } else {
        seekfree_assistant_interface_init(DEBUG_DISPLAY_DEVICE);
    }
}

void ui_update(const Ctrl_Input_t    *fb,
               const Nav_Input_t     *nav_input,
               const Nav_Output_t    *nav_output,
               const Vision_Result_t *vision)
{
    switch (g_page)
    {
        case UI_PAGE_DASHBOARD:
            dashboard_update(fb, nav_input, nav_output);
            break;
        case UI_PAGE_IMU_DEBUG:
            imu_debug_display_update();
            break;
        case UI_PAGE_NAV_DEBUG:
            nav_debug_display_update(nav_input);
            break;
        case UI_PAGE_PID_DEBUG:
            break;
        case UI_PAGE_REMOTE:
            remote_page_update(fb, nav_input, nav_output, vision);
            break;
    }
}
