#ifndef HMI_INPUT_HANDLER_H
#define HMI_INPUT_HANDLER_H

#include "zf_device_key.h"

typedef void (*input_cb_t)(void);

// 初始化：scan_period_ms 为调用 input_handler_tick 的周期（毫秒）
void input_handler_init(uint32 scan_period_ms);

// 周期性调用（与 scan_period_ms 匹配）
void input_handler_tick(void);

// 注册按键回调，on_short/on_long 可为 NULL
void input_handler_set_cb(key_index_enum key,
                          input_cb_t on_short,
                          input_cb_t on_long);

#endif
