#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include <SPI.h>

// IMU用SPIバス (HSPI: SCK=40, MISO=38, MOSI=39)
extern SPIClass imu_spi;

SPIClass& get_imu_spi();
void init_imu_spi();

#endif
