#include "battery.h"

void Battery::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);
}

float Battery::read_voltage() {
    int raw = analogRead(ADC_PIN);
    return (float)raw / ADC_SCALE * ADC_VREF * DIV_RATIO;
}
