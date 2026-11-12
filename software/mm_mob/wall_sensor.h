#ifndef WALL_SENSOR_H
#define WALL_SENSOR_H

#include <Arduino.h>

// 光センサ3個（右・前・左）
// robosweep_twilightと異なり、外付けADCではなくESP32内蔵ADCを使用する。
//
// ピンアサイン:
//   右センサ: EN=IO4,  AN=IO5  (ADC1_CH4)
//   前センサ: EN=IO6,  AN=IO7  (ADC1_CH6)
//   左センサ: EN=IO15, AN=IO16 (ADC2_CH5 ※WiFi非使用時のみ)

class WallSensor {
public:
    WallSensor();
    void begin();

    uint16_t right();  // 右センサ差分値
    uint16_t front();  // 前センサ差分値
    uint16_t left();   // 左センサ差分値

    void set_enabled(bool enabled);
    bool is_enabled() const;

private:
    bool enabled_ = false;

    static constexpr int R_EN_PIN  = 4;
    static constexpr int R_ADC_PIN = 5;
    static constexpr int F_EN_PIN  = 6;
    static constexpr int F_ADC_PIN = 7;
    static constexpr int L_EN_PIN  = 15;
    static constexpr int L_ADC_PIN = 16;

    // LED点灯後のAD安定待ち時間
    static constexpr uint32_t EN_DELAY_US = 100;

    uint16_t read_sensor(int en_pin, int adc_pin);
};

#endif
