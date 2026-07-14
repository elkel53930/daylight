#include <Arduino.h>
#include <SPI.h>
#include "encoder.h"

Encoder::Encoder() : encoder_spi(FSPI) {}

void Encoder::begin() {
    pinMode(RIGHT_CS_PIN, OUTPUT);
    pinMode(LEFT_CS_PIN, OUTPUT);
    digitalWrite(RIGHT_CS_PIN, HIGH);
    digitalWrite(LEFT_CS_PIN, HIGH);

    encoder_spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
}

uint16_t Encoder::read_right_angle() {
    return read_encoder(RIGHT_CS_PIN);
}

uint16_t Encoder::read_left_angle() {
    return read_encoder(LEFT_CS_PIN);
}

float Encoder::read_right_angle_degrees() {
    return read_right_angle() * DEGREES_PER_COUNT;
}

float Encoder::read_left_angle_degrees() {
    return read_left_angle() * DEGREES_PER_COUNT;
}

bool Encoder::read_both_angles(uint16_t &right_angle, uint16_t &left_angle) {
    right_angle = read_right_angle();
    left_angle  = read_left_angle();
    return (right_angle != 0xFFFF && left_angle != 0xFFFF);
}

uint16_t Encoder::read_encoder(int cs_pin) {
    // AS5047 角度レジスタ読み取りコマンド (0x7FFE)
    const uint8_t w_buf[2] = {0x7F, 0xFE};
    uint8_t r_buf[2] = {0, 0};

    encoder_spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE));
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    r_buf[0] = encoder_spi.transfer(w_buf[0]);
    r_buf[1] = encoder_spi.transfer(w_buf[1]);
    digitalWrite(cs_pin, HIGH);
    encoder_spi.endTransaction();

    // 下位14bitのみ使用
    uint16_t result = ((uint16_t)r_buf[0] << 8) | r_buf[1];
    return result & 0x3FFF;
}
