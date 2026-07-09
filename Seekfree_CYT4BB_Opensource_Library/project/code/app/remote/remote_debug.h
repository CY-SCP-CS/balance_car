#ifndef APP_REMOTE_REMOTE_DEBUG_H
#define APP_REMOTE_REMOTE_DEBUG_H

#include "zf_common_typedef.h"
#include "seekfree_assistant.h"

#define REMOTE_DEBUG_PARAM_CHANNELS SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT

void remote_debug_init(void);
void remote_debug_bind(uint8 channel, float *target);
void remote_debug_send_param(uint8 channel, float value);

#endif
