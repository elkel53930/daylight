#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// モータードライバ
//   PWM周波数: 40kHz, 10bit分解能 (0–1023)
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

private:
    static constexpr int RIGHT_PWM_PIN = 48;
    static constexpr int RIGHT_DIR_PIN = 45;
    static constexpr int LEFT_PWM_PIN  = 21;
    static constexpr int LEFT_DIR_PIN  = 47;

    static constexpr int     PWM_FREQUENCY = 40000;  // 40kHz
    static constexpr uint8_t PWM_RESOLUTION = 10;    // 10bit (0–1023)
    static constexpr int16_t PWM_MAX = (1 << PWM_RESOLUTION) - 1;  // 1023

    void set_motor(int16_t speed, int pwm_pin, int dir_pin);
};

#endif
