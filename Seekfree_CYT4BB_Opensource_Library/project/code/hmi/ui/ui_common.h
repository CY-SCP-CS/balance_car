#ifndef HMI_UI_COMMON_H
#define HMI_UI_COMMON_H

#include "zf_common_typedef.h"
#include "../../common/types.h"
#include "../../app/navigation/nav_engine.h"
#include "../../app/vision/vision_pipeline.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"

#define UI_RAD_TO_DEG    (180.0f / 3.14159265f)

// 通信方式：按实际连接方式修改
// SEEKFREE_ASSISTANT_DEBUG_UART    → USB/调试串口
// SEEKFREE_ASSISTANT_WIRELESS_UART → 无线串口模块
// SEEKFREE_ASSISTANT_BLE6A20       → 蓝牙透传模块
#define DEBUG_DISPLAY_DEVICE    SEEKFREE_ASSISTANT_DEBUG_UART

/* 每个 hmi/ui 页面 update 函数的统一入参。仅供 hmi/ui 内部使用：
 * ui_manager.c 从 ui_update() 的 4 个独立参数组装出一个 UI_Frame_t，
 * 再分发给当前激活页面。 */
typedef struct {
    const Ctrl_Input_t    *fb;
    const Nav_Input_t     *nav_input;
    const Nav_Output_t    *nav_output;
    const Vision_Result_t *vision;
} UI_Frame_t;

/* 除 UI_PAGE_REMOTE（需要自己的 WiFi/摄像头传输初始化）外，
 * 其余页面共用这个初始化。 */
static inline void ui_debug_display_init(void)
{
    seekfree_assistant_interface_init(DEBUG_DISPLAY_DEVICE);
}

/* nav 处于 safety_stop 时显示 -1，否则显示当前路径段序号。 */
static inline float ui_segment_or_stop(const Nav_Output_t *nav_output, uint8 segment_index)
{
    return (nav_output != NULL && nav_output->safety_stop) ? -1.0f : (float)segment_index;
}

/* 填充 6 通道示波器数据包并发送。 */
static inline void ui_scope6_send(float c0, float c1, float c2,
                                  float c3, float c4, float c5)
{
    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] = c0;
    seekfree_assistant_oscilloscope_data.data[1] = c1;
    seekfree_assistant_oscilloscope_data.data[2] = c2;
    seekfree_assistant_oscilloscope_data.data[3] = c3;
    seekfree_assistant_oscilloscope_data.data[4] = c4;
    seekfree_assistant_oscilloscope_data.data[5] = c5;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

#endif
