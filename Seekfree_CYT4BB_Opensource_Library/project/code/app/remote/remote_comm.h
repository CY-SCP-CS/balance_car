#ifndef APP_REMOTE_REMOTE_COMM_H
#define APP_REMOTE_REMOTE_COMM_H

#include "zf_common_typedef.h"
#include "../../common/types.h"

typedef struct {
    int16 joystick[4];
    // 0-左边 左右
    // 1-左边 上下
    // 2-右边 左右
    // 3-右边 上下

    uint8 key[4];
    // 0-摇杆左边
    // 1-摇杆右边
    // 2-侧向按键左边
    // 3-侧向按键右边

    uint8 switch_key[4];
    // 0-左边_1
    // 1-左边_2
    // 2-右边_1
    // 3-右边_2
    bool connected;
    bool frame_updated;
} Remote_State_t;

void remote_comm_init(void);
void remote_comm_update(Ctrl_Input_t *ctrl);
const Remote_State_t *remote_comm_get_state(void);

#endif
