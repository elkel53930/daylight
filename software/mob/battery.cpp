#include "battery.h"

void Battery::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);
}

float Battery::read_voltage() {
    // eFuse の工場キャリブレーションで減衰率・個体差を補正した mV 値を使う
    uint32_t mv = analogReadMilliVolts(ADC_PIN);
    return (float)mv / 1000.0f * DIV_RATIO;
}
