#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"

// RCサーボ PWM ドライバ (MG90S)
//   IO1 → サーボ SIG 線
//   PWM周波数: 50Hz (20ms周期)
//   パルス幅:  500µs (0°) 〜 2500µs (180°)
//   MCPWM group 1 使用 (group 0 はモーターが使用中)

class Servo {
public:
    Servo();
    void begin();

    void set_angle(uint8_t angle);  // 0–180 度

    // トルクオフ（脱力）。パルス出力をLowに強制固定し、サーボへの
    // 位置指令パルスを止める（PWM制御サーボにはトルクON/OFFレジスタが
    // 無いため、パルスの有無でトルクを制御する）。次のset_angle()呼び出しで
    // 自動的に解除され通常のPWM出力に戻る。
    void detach();

private:
    static constexpr int      SIGNAL_PIN          = 1;
    static constexpr uint32_t TIMER_RESOLUTION_HZ = 1000000;  // 1MHz → 1µs/tick
    static constexpr uint32_t PERIOD_TICKS         = 20000;   // 20ms = 50Hz
    static constexpr uint32_t PULSE_MIN_US         = 500;     // 0°
    static constexpr uint32_t PULSE_MAX_US         = 2500;    // 180°

    mcpwm_timer_handle_t _timer;
    mcpwm_oper_handle_t  _oper;
    mcpwm_cmpr_handle_t  _cmpr;
    mcpwm_gen_handle_t   _gen;
};

#endif
