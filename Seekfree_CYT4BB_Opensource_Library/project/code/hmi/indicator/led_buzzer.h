#ifndef HMI_LED_BUZZER_H
#define HMI_LED_BUZZER_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

// ---- 引脚配置（根据实际硬件修改）----
#define BUZZER_PWM_CH    TCPWM_CH29_P19_4
#define BUZZER_FREQ_HZ   2000
#define BUZZER_DUTY      5000    // 50%
// -------------------------------------

typedef enum {
    BEEP_SHORT,     // 100ms 单次短鸣
    BEEP_LONG,      // 500ms 长鸣
    BEEP_DOUBLE,    // 双鸣：100ms-50ms-100ms
    BEEP_TRIPLE,    // 三短鸣：3×(100ms on / 50ms off)
    BEEP_DOUBLE_LONG, // 双长鸣：500ms-100ms-500ms
    BEEP_ERROR,     // 三连鸣：3×(100ms on / 50ms off)
} Beep_Pattern_t;

void  led_buzzer_init   (void);
void  led_buzzer_tick   (uint32 period_ms);  // 周期性调用，单位毫秒

void  buzzer_beep       (Beep_Pattern_t pattern);
void  buzzer_stop       (void);
uint8 buzzer_is_active  (void);

#endif
