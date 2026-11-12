#include <Arduino.h>
#include "fan.h"

Fan::Fan() {}

void Fan::begin() {
    ledcAttach(PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcWrite(PWM_PIN, 0);
}

void Fan::set_speed(uint8_t speed) {
    ledcWrite(PWM_PIN, speed);
}

void Fan::stop() {
    ledcWrite(PWM_PIN, 0);
}
