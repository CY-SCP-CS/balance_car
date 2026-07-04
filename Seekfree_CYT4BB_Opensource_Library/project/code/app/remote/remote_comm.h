#ifndef APP_REMOTE_REMOTE_COMM_H
#define APP_REMOTE_REMOTE_COMM_H

#include "zf_common_typedef.h"
#include "../../common/types.h"

typedef struct {
    float joystick[4];
    uint8 key[4];
    uint8 switch_key[4];
    bool connected;
    bool frame_updated;
} Remote_State_t;

void remote_comm_init(void);
void remote_comm_update(Ctrl_Input_t *ctrl);
const Remote_State_t *remote_comm_get_state(void);

#endif
