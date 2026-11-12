#include "wall_sensor.h"

WallSensor::WallSensor() {}

void WallSensor::begin() {
    pinMode(R_EN_PIN, OUTPUT);
    pinMode(F_EN_PIN, OUTPUT);
    pinMode(L_EN_PIN, OUTPUT);

    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(F_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);

    // 12bit分解能, 11dB減衰（0〜3.3V範囲）
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    enabled_ = false;
}

void WallSensor::set_enabled(bool enabled) {
    enabled_ = enabled;
    if (!enabled_) {
        digitalWrite(R_EN_PIN, LOW);
        digitalWrite(F_EN_PIN, LOW);
        digitalWrite(L_EN_PIN, LOW);
    }
}

bool WallSensor::is_enabled() const {
    return enabled_;
}

// LED OFF時とON時の差分を返す（環境光キャンセル）
uint16_t WallSensor::read_sensor(int en_pin, int adc_pin) {
    if (!enabled_) return 0;

    uint16_t off = (uint16_t)analogRead(adc_pin);
    digitalWrite(en_pin, HIGH);
    ets_delay_us(EN_DELAY_US);
    uint16_t on = (uint16_t)analogRead(adc_pin);
    digitalWrite(en_pin, LOW);

    return (on > off) ? (on - off) : 0;
}

uint16_t WallSensor::right() { return read_sensor(R_EN_PIN, R_ADC_PIN); }
uint16_t WallSensor::front() { return read_sensor(F_EN_PIN, F_ADC_PIN); }
uint16_t WallSensor::left()  { return read_sensor(L_EN_PIN, L_ADC_PIN); }
