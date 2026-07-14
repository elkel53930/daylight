#include <Arduino.h>
#include "motor.h"

Motor::Motor() {}

void Motor::begin() {
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN, LOW);

    ledcAttach(RIGHT_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(LEFT_PWM_PIN,  PWM_FREQUENCY, PWM_RESOLUTION);

    ledcWrite(RIGHT_PWM_PIN, 0);
    ledcWrite(LEFT_PWM_PIN,  0);
}

void Motor::set_right(int16_t speed) {
    set_motor(speed, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
}

void Motor::set_left(int16_t speed) {
    set_motor(speed, LEFT_PWM_PIN, LEFT_DIR_PIN);
}

void Motor::stop() {
    ledcWrite(RIGHT_PWM_PIN, 0);
    ledcWrite(LEFT_PWM_PIN,  0);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN,  LOW);
}

void Motor::set_motor(int16_t speed, int pwm_pin, int dir_pin) {
    if (speed >  PWM_MAX) speed =  PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed >= 0) {
        digitalWrite(dir_pin, HIGH);  // 正転 (CW)
        ledcWrite(pwm_pin, (uint32_t)speed);
    } else {
        digitalWrite(dir_pin, LOW);   // 逆転 (CCW)
        ledcWrite(pwm_pin, (uint32_t)(-speed));
    }
}
