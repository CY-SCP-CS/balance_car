#ifndef HMI_PAGE_REMOTE_H
#define HMI_PAGE_REMOTE_H

#include "zf_common_typedef.h"
#include "ui_common.h"
#include "../../app/remote/remote_debug.h"

#define REMOTE_WIFI_SSID        "BNGU"
#define REMOTE_WIFI_PASSWORD    "12345678"
#define REMOTE_TARGET_IP        "192.168.1.102"
#define REMOTE_TARGET_PORT      "8086"
#define REMOTE_LOCAL_PORT       "6666"

#define REMOTE_PARAM_CHANNELS   REMOTE_DEBUG_PARAM_CHANNELS

void page_remote_init(void);
void page_remote_update(const UI_Frame_t *frame);

#endif
