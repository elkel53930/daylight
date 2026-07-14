#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>

// LSM6DSR 6軸IMU (ジャイロ + 加速度)
// SPIバス: imu_spi (HSPI: SCK=40, MISO=38, MOSI=39)
//   CS   = IO41
//   SCK  = IO40
//   MOSI = IO39
//   MISO = IO38

class IMU {
public:
    IMU(SPIClass& spi_bus);

    // 初期化 (true: 成功)
    bool begin();

    // Z軸ジャイロ生データ読み取り
    int16_t read_gyro_z();

    // 生データ → rad/s 変換 (FS=±1000dps)
    float convert_gyro_z_to_radps(int16_t raw);

    // デバッグ用
    uint8_t read_who_am_i();
    uint8_t read_status();

private:
    SPIClass& spi;

    static constexpr int IMU_CS = 41;

    // ジャイロ感度: FS=±1000dps → 35mdps/LSB
    static constexpr float GYRO_SENSITIVITY_RADPS =
        0.035f * 3.14159265359f / 180.0f;  // rad/s/LSB

    // LSM6DSR レジスタアドレス
    static constexpr uint8_t WHO_AM_I_REG   = 0x0F;
    static constexpr uint8_t WHO_AM_I_VALUE = 0x6B;
    static constexpr uint8_t CTRL1_XL       = 0x10;
    static constexpr uint8_t CTRL2_G        = 0x11;
    static constexpr uint8_t CTRL3_C        = 0x12;
    static constexpr uint8_t CTRL6_C        = 0x15;
    static constexpr uint8_t CTRL7_G        = 0x16;
    static constexpr uint8_t STATUS_REG     = 0x1E;
    static constexpr uint8_t OUTZ_L_G       = 0x26;

    void imu_transfer(uint8_t* r_buf, const uint8_t* w_buf, size_t len);
};

#endif
