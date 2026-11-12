#include "spi_manager.h"

// IMU専用SPIバス
SPIClass imu_spi(HSPI);

SPIClass& get_imu_spi() {
    return imu_spi;
}

void init_imu_spi() {
    // SCK=40, MISO=38, MOSI=39, CS=-1 (ソフトウェア制御)
    imu_spi.begin(40, 38, 39, -1);
}
