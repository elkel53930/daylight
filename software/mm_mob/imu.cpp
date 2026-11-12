#include <Arduino.h>
#include <SPI.h>
#include "imu.h"

IMU::IMU(SPIClass& spi_bus) : spi(spi_bus) {
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);
}

bool IMU::begin() {
    uint8_t r_buf[2] = {0, 0};

    // ソフトウェアリセット (CTRL3_C: SW_RESET)
    const uint8_t reset_cmd[2] = {0x12, 0x01};
    imu_transfer(r_buf, reset_cmd, 2);
    delay(50);

    // ジャイロ設定: ODR=416Hz, FS=±1000dps (CTRL2_G = 0x68)
    const uint8_t gyro_cfg[2] = {0x11, 0x68};
    imu_transfer(r_buf, gyro_cfg, 2);

    // 加速度設定: ODR=416Hz, FS=±4g (CTRL1_XL = 0x68)
    const uint8_t accel_cfg[2] = {0x10, 0x68};
    imu_transfer(r_buf, accel_cfg, 2);

    // BDU + IF_INC 有効化 (CTRL3_C = 0x44)
    const uint8_t ctrl3_cfg[2] = {0x12, 0x44};
    imu_transfer(r_buf, ctrl3_cfg, 2);

    // CTRL6_C: ジャイロ ハイパフォーマンスモード有効
    const uint8_t ctrl6_cfg[2] = {0x15, 0x00};
    imu_transfer(r_buf, ctrl6_cfg, 2);

    // CTRL7_G: LPF1 デフォルト設定
    const uint8_t ctrl7_cfg[2] = {0x16, 0x00};
    imu_transfer(r_buf, ctrl7_cfg, 2);

    delay(100);
    return true;
}

int16_t IMU::read_gyro_z() {
    // OUTZ_L_G (0x26) + OUTZ_H_G (0x27) を連続読み取り
    // 読み取りビット(0x80) + auto-increment(IF_INC有効)で2バイト取得
    const uint8_t w_buf[3] = {0xA6, 0xFF, 0xFF};  // 0x26 | 0x80
    uint8_t r_buf[3] = {0, 0, 0};
    imu_transfer(r_buf, w_buf, 3);

    // r_buf[0]=ダミー, r_buf[1]=LSB, r_buf[2]=MSB (リトルエンディアン)
    return (int16_t)(((uint16_t)r_buf[2] << 8) | r_buf[1]);
}

float IMU::convert_gyro_z_to_radps(int16_t raw) {
    return (float)raw * GYRO_SENSITIVITY_RADPS;
}

uint8_t IMU::read_who_am_i() {
    const uint8_t w_buf[2] = {0x8F, 0xFF};  // 0x0F | 0x80
    uint8_t r_buf[2] = {0, 0};
    imu_transfer(r_buf, w_buf, 2);
    return r_buf[1];
}

uint8_t IMU::read_status() {
    const uint8_t w_buf[2] = {0x9E, 0xFF};  // 0x1E | 0x80
    uint8_t r_buf[2] = {0, 0};
    imu_transfer(r_buf, w_buf, 2);
    return r_buf[1];
}

void IMU::imu_transfer(uint8_t* r_buf, const uint8_t* w_buf, size_t len) {
    spi.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(50);
    for (size_t i = 0; i < len; i++) {
        r_buf[i] = spi.transfer(w_buf[i]);
    }
    delayMicroseconds(50);
    digitalWrite(IMU_CS, HIGH);
    spi.endTransaction();
}
