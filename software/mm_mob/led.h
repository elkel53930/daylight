#ifndef LED_H
#define LED_H

#include <Arduino.h>

// オンボード LED
//   GREEN = IO20
//   RED   = IO3

class Led {
public:
    Led();
    void begin();

    void green_on();
    void green_off();
    void red_on();   // T互換
    void red_off();  // T互換
    
    // 赤と緑を交互に切り替え (呼ぶたびに状態が反転)
    void toggle_rx();

private:
    static constexpr int GREEN_PIN = 20;
    static constexpr int RED_PIN   = 3;

    bool _green_active;  // true = 現在緑が点灯中
};

#endif
