#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"

// モータードライバ (MCPWM)
//   PWM周波数: ~39kHz (40 MHz / 1024 ticks), 速度分解能 0–1023
//   速度指定: -1023 〜 +1023 (負=逆転)
//
//   PWM L    = IO21
//   CWCCW L  = IO47
//   PWM R    = IO48
//   CWCCW R  = IO45

class Motor {
public:
    Motor();
    void begin();

    void set_right(int16_t speed);  // 右モーター (-1023〜+1023)
    void set_left(int16_t speed);   // 左モーター (-1023〜+1023)
    void stop();                    // 両モーター停止

    // 両モーターの最後の指令が共に0か(=停止中)。外力が無い前提では停止中は
    // 機体が動かず角度も変わらないので、ジャイロ角度積分の凍結判定に使う
    // (2026-08-03、ユーザー指摘)。
    bool is_stopped() const { return last_right_ == 0 && last_left_ == 0; }

private:
    int16_t last_right_ = 0;  // 最後に set_right した速度指令
    int16_t last_left_  = 0;  // 最後に set_left した速度指令

    static constexpr int RIGHT_PWM_PIN = 48;
    static constexpr int RIGHT_DIR_PIN = 45;
    static constexpr int LEFT_PWM_PIN  = 21;
    static constexpr int LEFT_DIR_PIN  = 47;

    static constexpr uint32_t TIMER_RESOLUTION_HZ = 40000000;  // 40 MHz
    static constexpr uint32_t PERIOD_TICKS         = 1024;      // ~39 kHz
    static constexpr int16_t  PWM_MAX              = 1023;

    mcpwm_timer_handle_t _timer;
    mcpwm_oper_handle_t  _oper_right;
    mcpwm_oper_handle_t  _oper_left;
    mcpwm_cmpr_handle_t  _cmpr_right;
    mcpwm_cmpr_handle_t  _cmpr_left;
    mcpwm_gen_handle_t   _gen_right;
    mcpwm_gen_handle_t   _gen_left;

    void set_motor(int16_t speed, mcpwm_cmpr_handle_t cmpr, int dir_pin);
};

#endif
