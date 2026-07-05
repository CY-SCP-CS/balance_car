#include "ui_manager.h"

#include "page_dashboard.h"
#include "page_imu_debug.h"
#include "page_nav_debug.h"
#include "page_remote.h"

#include "ui_common.h"

typedef void (*ui_page_update_fn)(const UI_Frame_t *frame);

static UI_Page_t g_page;

static const ui_page_update_fn g_page_update_table[UI_PAGE_COUNT] = {
    [UI_PAGE_DASHBOARD] = page_dashboard_update,
    [UI_PAGE_IMU_DEBUG] = page_imu_debug_update,
    [UI_PAGE_NAV_DEBUG] = page_nav_debug_update,
    [UI_PAGE_REMOTE]    = page_remote_update,
};

void ui_init(UI_Page_t page)
{
    g_page = page;

    if (page == UI_PAGE_REMOTE) {
        page_remote_init();
    } else {
        ui_debug_display_init();
    }
}

void ui_update(const Ctrl_Input_t    *fb,
               const Nav_Input_t     *nav_input,
               const Nav_Output_t    *nav_output,
               const Vision_Result_t *vision)
{
    const UI_Frame_t frame = {
        .fb         = fb,
        .nav_input  = nav_input,
        .nav_output = nav_output,
        .vision     = vision,
    };

    if (g_page < UI_PAGE_COUNT && g_page_update_table[g_page] != NULL) {
        g_page_update_table[g_page](&frame);
    }
}
