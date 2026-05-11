#ifndef HMI_PAGE_REMOTE_H
#define HMI_PAGE_REMOTE_H

#include "../../common/types.h"
#include "../../app/navigation/nav_engine.h"
#include "../../app/vision/vision_pipeline.h"

#define REMOTE_WIFI_SSID        "your_ssid"
#define REMOTE_WIFI_PASSWORD    "your_password"
#define REMOTE_TARGET_IP        "192.168.137.1"
#define REMOTE_TARGET_PORT      "8086"
#define REMOTE_LOCAL_PORT       "6666"

void remote_page_init(void);
void remote_page_update(const Ctrl_Input_t    *fb,
                        const Nav_Input_t     *nav_input,
                        const Nav_Output_t    *nav_output,
                        const Vision_Result_t *vision);

#endif
