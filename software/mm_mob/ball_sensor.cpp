#include <Arduino.h>
#include "ball_sensor.h"

BallSensor::BallSensor(uint16_t threshold)
    : threshold_(threshold)
{}

void BallSensor::begin() {
    pinMode(ADC_PIN, INPUT);
    analogReadResolution(12);  // 12bit (0–4095)
}

uint16_t BallSensor::read_raw() {
    return (uint16_t)analogRead(ADC_PIN);
}

bool BallSensor::detect() {
    return read_raw() >= threshold_;
}

void BallSensor::set_threshold(uint16_t threshold) {
    threshold_ = threshold;
}

uint16_t BallSensor::get_threshold() const {
    return threshold_;
}
