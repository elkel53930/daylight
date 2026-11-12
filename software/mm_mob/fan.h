#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

// 吸引ファン PWM ドライバ
//   IO2 → NchFET ゲート → ファンモーター
//   PWM周波数: 40kHz, 8bit分解能 (0–255)
//   set_speed(0) = 停止, set_speed(255) = フル

class Fan {
public:
    Fan();
    void begin();

    void set_speed(uint8_t speed);  // 0–255
    void stop();

private:
    static constexpr int     PWM_PIN       = 2;
    static constexpr int     PWM_FREQUENCY = 40000;  // 40kHz
    static constexpr uint8_t PWM_RESOLUTION = 8;     // 8bit (0–255)
};

#endif
