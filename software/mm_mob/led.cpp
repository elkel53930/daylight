#include <Arduino.h>
#include "led.h"

Led::Led() : _green_active(false) {}

void Led::begin() {
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(RED_PIN,   OUTPUT);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(RED_PIN,   LOW);
}

void Led::green_on()  { digitalWrite(GREEN_PIN, HIGH); }
void Led::green_off() { digitalWrite(GREEN_PIN, LOW);  }
void Led::red_on()    { digitalWrite(RED_PIN,   HIGH); }
void Led::red_off()   { digitalWrite(RED_PIN,   LOW);  }

void Led::toggle_rx() {
    if (_green_active) {
        digitalWrite(GREEN_PIN, LOW);
        digitalWrite(RED_PIN,   HIGH);
        _green_active = false;
    } else {
        digitalWrite(RED_PIN,   LOW);
        digitalWrite(GREEN_PIN, HIGH);
        _green_active = true;
    }
}
