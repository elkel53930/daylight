#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// バッテリー電圧監視
// IO13の電圧を11倍するとバッテリー電圧になる (ADC2_CH2 ※WiFi非使用時のみ)

class Battery {
public:
    void begin();
    float read_voltage();  // バッテリー電圧 [V]

private:
    static constexpr int   ADC_PIN    = 13;
    static constexpr float DIV_RATIO  = 11.0f;   // 分圧比の逆数
    static constexpr float ADC_VREF   = 3.3f;    // [V]
    static constexpr float ADC_SCALE  = 4095.0f; // 12bit
};

#endif
