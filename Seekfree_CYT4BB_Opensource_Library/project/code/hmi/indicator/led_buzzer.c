#include "led_buzzer.h"

// ---- 蜂鸣器模式步骤表 ----
// 每步 {持续时间 ms, 是否响}；duration=0 表示结束
typedef struct {
    uint16 duration_ms;
    uint8  on;
} Beep_Step_t;

static const Beep_Step_t s_pat_short[]  = {{100, 1}, {0, 0}};
static const Beep_Step_t s_pat_long[]   = {{500, 1}, {0, 0}};
static const Beep_Step_t s_pat_double[] = {{100, 1}, {50, 0}, {100, 1}, {0, 0}};
static const Beep_Step_t s_pat_triple[] = {{100, 1}, {50, 0}, {100, 1}, {50, 0}, {100, 1}, {0, 0}};
static const Beep_Step_t s_pat_double_long[] = {{500, 1}, {100, 0}, {500, 1}, {0, 0}};

static const Beep_Step_t *s_patterns[] = {
    s_pat_short,
    s_pat_long,
    s_pat_double,
    s_pat_triple,
    s_pat_double_long,
    s_pat_triple,
};

// ---- 蜂鸣器状态 ----
static const Beep_Step_t *s_cur_pat   = NULL;
static uint8              s_step      = 0;
static uint32             s_elapsed   = 0;

// ----------------------------------------------------------------
static void buzzer_hw_set(uint8 on)
{
    if (on) {
        pwm_set_duty(BUZZER_PWM_CH, BUZZER_DUTY);
    } else {
        pwm_set_duty(BUZZER_PWM_CH, 0);
    }
}

// ----------------------------------------------------------------
void led_buzzer_init(void)
{
    pwm_init(BUZZER_PWM_CH, BUZZER_FREQ_HZ, 0);
    s_cur_pat = NULL;
    s_step    = 0;
    s_elapsed = 0;
}

// ----------------------------------------------------------------
void led_buzzer_tick(uint32 period_ms)
{
    if (s_cur_pat == NULL) return;

    const Beep_Step_t *step = &s_cur_pat[s_step];
    if (step->duration_ms == 0) {
        // 模式播放完毕
        buzzer_hw_set(0);
        s_cur_pat = NULL;
        return;
    }

    s_elapsed += period_ms;
    if (s_elapsed >= step->duration_ms) {
        s_elapsed -= step->duration_ms;  // 保留溢出时间，减少累积误差
        s_step++;
        step = &s_cur_pat[s_step];
        if (step->duration_ms == 0) {
            buzzer_hw_set(0);
            s_cur_pat = NULL;
        } else {
            buzzer_hw_set(step->on);
        }
    }
}

// ----------------------------------------------------------------
void buzzer_beep(Beep_Pattern_t pattern)
{
    s_cur_pat = s_patterns[pattern];
    s_step    = 0;
    s_elapsed = 0;
    buzzer_hw_set(s_cur_pat[0].on);
}

void buzzer_stop(void)
{
    s_cur_pat = NULL;
    s_step    = 0;
    s_elapsed = 0;
    buzzer_hw_set(0);
}

uint8 buzzer_is_active(void)
{
    return (s_cur_pat != NULL) ? 1u : 0u;
}
