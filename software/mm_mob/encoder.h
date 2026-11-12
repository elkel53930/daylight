#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <SPI.h>

// AS5047 磁気エンコーダ (×2)
// 専用SPIバス (FSPI)
//   R CS  = IO46
//   L CS  = IO9
//   SCK   = IO10
//   MISO  = IO11
//   MOSI  = IO12

class Encoder {
public:
    Encoder();
    void begin();

    uint16_t read_right_angle();           // 右エンコーダ角度 (0–16383)
    uint16_t read_left_angle();            // 左エンコーダ角度 (0–16383)
    float    read_right_angle_degrees();   // 右エンコーダ角度 [deg]
    float    read_left_angle_degrees();    // 左エンコーダ角度 [deg]

    // 両エンコーダを同時読み取り (0xFFFF はエラー)
    bool read_both_angles(uint16_t &right_angle, uint16_t &left_angle);

private:
    SPIClass encoder_spi;

    static constexpr int SCK_PIN      = 10;
    static constexpr int MISO_PIN     = 11;
    static constexpr int MOSI_PIN     = 12;
    static constexpr int RIGHT_CS_PIN = 46;
    static constexpr int LEFT_CS_PIN  = 9;

    static constexpr uint32_t SPI_FREQUENCY = 10000000;  // 10MHz
    static constexpr uint8_t  SPI_MODE      = SPI_MODE1; // AS5047はMODE1

    static constexpr uint16_t ENCODER_RESOLUTION = 16384; // 14bit
    static constexpr float DEGREES_PER_COUNT = 360.0f / ENCODER_RESOLUTION;

    uint16_t read_encoder(int cs_pin);
};

#endif
