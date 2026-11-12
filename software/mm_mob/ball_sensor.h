#ifndef BALL_SENSOR_H
#define BALL_SENSOR_H

#include <Arduino.h>

// ボールセンサ (光反射式)
//   IO14 (ADC1_CH3) → センサ出力電圧
//   detect() がtrueのとき、ボールが検出されている
//   しきい値はコンストラクタで変更可能 (デフォルト2048 = 約1.65V)

class BallSensor {
public:
    explicit BallSensor(uint16_t threshold = 2048);
    void begin();

    uint16_t read_raw();     // ADC生値 (0–4095)
    bool     detect();       // しきい値以上ならtrue

    void set_threshold(uint16_t threshold);
    uint16_t get_threshold() const;

private:
    static constexpr int ADC_PIN = 14;  // ADC1_CH3

    uint16_t threshold_;
};

#endif
