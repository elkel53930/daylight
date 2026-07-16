#include <Arduino.h>
#include "servo.h"

Servo::Servo()
    : _timer(nullptr), _oper(nullptr), _cmpr(nullptr), _gen(nullptr)
{}

void Servo::begin() {
    // タイマー: 1MHz / 20000 ticks = 50Hz
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id      = 1;  // group 0 はモーターが使用中
    timer_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = TIMER_RESOLUTION_HZ;
    timer_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    timer_cfg.period_ticks  = PERIOD_TICKS;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &_timer));

    // オペレーター
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = 1;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper, _timer));

    // コンパレーター (TEZ タイミングで比較値を更新)
    mcpwm_comparator_config_t cmpr_cfg = {};
    cmpr_cfg.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper, &cmpr_cfg, &_cmpr));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr, PULSE_MIN_US));

    // ジェネレーター: タイマー零でHigh、コンペア一致でLow → 正パルス
    mcpwm_generator_config_t gen_cfg = {};
    gen_cfg.gen_gpio_num = SIGNAL_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper, &gen_cfg, &_gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                      MCPWM_TIMER_EVENT_EMPTY,
                                      MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(_gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                        _cmpr,
                                        MCPWM_GEN_ACTION_LOW)));

    // タイマー起動
    ESP_ERROR_CHECK(mcpwm_timer_enable(_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP));
}

void Servo::set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    // 0°→500µs, 180°→2500µs の線形補間
    uint32_t pulse_us = PULSE_MIN_US
                      + (uint32_t)angle * (PULSE_MAX_US - PULSE_MIN_US) / 180;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr, pulse_us));
}
