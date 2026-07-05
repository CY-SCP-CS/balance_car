#ifndef HMI_PAGE_REMOTE_H
#define HMI_PAGE_REMOTE_H

#include "zf_common_typedef.h"
#include "ui_common.h"

#define REMOTE_WIFI_SSID        "your_ssid"
#define REMOTE_WIFI_PASSWORD    "your_password"
#define REMOTE_TARGET_IP        "192.168.137.1"
#define REMOTE_TARGET_PORT      "8086"
#define REMOTE_LOCAL_PORT       "6666"

#define REMOTE_PARAM_CHANNELS   8u

void page_remote_init(void);
void remote_param_bind(uint8 channel, float *target);
void page_remote_update(const UI_Frame_t *frame);

#endif
