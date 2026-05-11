#include "input_handler.h"

static input_cb_t s_short_cb[KEY_NUMBER];
static input_cb_t s_long_cb[KEY_NUMBER];

// 防止长按期间重复触发：记录本次按压是否已派发长按回调
static uint8 s_long_fired[KEY_NUMBER];

void input_handler_init(uint32 scan_period_ms)
{
    key_init(scan_period_ms);
    for (uint8 i = 0; i < KEY_NUMBER; i++) {
        s_short_cb[i]   = NULL;
        s_long_cb[i]    = NULL;
        s_long_fired[i] = 0;
    }
}

void input_handler_tick(void)
{
    key_scanner();

    for (uint8 i = 0; i < KEY_NUMBER; i++) {
        key_state_enum state = key_get_state((key_index_enum)i);

        switch (state) {
            case KEY_SHORT_PRESS:
                if (s_short_cb[i]) s_short_cb[i]();
                s_long_fired[i] = 0;
                key_clear_state((key_index_enum)i);
                break;

            case KEY_LONG_PRESS:
                if (!s_long_fired[i]) {
                    if (s_long_cb[i]) s_long_cb[i]();
                    s_long_fired[i] = 1;
                }
                break;

            case KEY_RELEASE:
                s_long_fired[i] = 0;
                break;

            default:
                break;
        }
    }
}

void input_handler_set_cb(key_index_enum key,
                          input_cb_t on_short,
                          input_cb_t on_long)
{
    s_short_cb[key] = on_short;
    s_long_cb[key]  = on_long;
}
