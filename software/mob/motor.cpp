#include <Arduino.h>
#include "motor.h"

Motor::Motor()
    : _timer(nullptr),
      _oper_right(nullptr), _oper_left(nullptr),
      _cmpr_right(nullptr), _cmpr_left(nullptr),
      _gen_right(nullptr),  _gen_left(nullptr)
{}

static void configure_generator(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmpr) {
    // カウントアップ中: タイマー零でHigh、コンペア一致でLow → 標準PWM
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                      MCPWM_TIMER_EVENT_EMPTY,
                                      MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                        cmpr,
                                        MCPWM_GEN_ACTION_LOW)));
}

void Motor::begin() {
    // 方向ピン初期化
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN, LOW);

    // タイマー: 40 MHz / 1024 ticks ≈ 39 kHz
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id      = 0;
    timer_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = TIMER_RESOLUTION_HZ;
    timer_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    timer_cfg.period_ticks  = PERIOD_TICKS;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &_timer));

    // オペレーター (右・左 それぞれ1つ)
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &_oper_right));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &_oper_left));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper_right, _timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper_left,  _timer));

    // コンパレーター (TEZ タイミングで比較値を更新)
    mcpwm_comparator_config_t cmpr_cfg = {};
    cmpr_cfg.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper_right, &cmpr_cfg, &_cmpr_right));
    ESP_ERROR_CHECK(mcpwm_new_comparator(_oper_left,  &cmpr_cfg, &_cmpr_left));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr_right, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr_left,  0));

    // ジェネレーター (GPIO割り当て + PWMアクション設定)
    mcpwm_generator_config_t gen_cfg = {};
    gen_cfg.gen_gpio_num = RIGHT_PWM_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper_right, &gen_cfg, &_gen_right));
    gen_cfg.gen_gpio_num = LEFT_PWM_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(_oper_left,  &gen_cfg, &_gen_left));

    configure_generator(_gen_right, _cmpr_right);
    configure_generator(_gen_left,  _cmpr_left);

    // タイマー起動
    ESP_ERROR_CHECK(mcpwm_timer_enable(_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP));
}

void Motor::set_right(int16_t speed) {
    set_motor(speed, _cmpr_right, RIGHT_DIR_PIN);
}

void Motor::set_left(int16_t speed) {
    set_motor(speed, _cmpr_left, LEFT_DIR_PIN);
}

void Motor::stop() {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr_right, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr_left,  0));
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN,  LOW);
}

void Motor::set_motor(int16_t speed, mcpwm_cmpr_handle_t cmpr, int dir_pin) {
    if (speed >  PWM_MAX) speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed >= 0) {
        digitalWrite(dir_pin, LOW);   // 正転 (CCW) — 実機の前進方向に合わせて反転(2026-07-18)
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, (uint32_t)speed));
    } else {
        digitalWrite(dir_pin, HIGH);  // 逆転 (CW)
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, (uint32_t)(-speed)));
    }
}
