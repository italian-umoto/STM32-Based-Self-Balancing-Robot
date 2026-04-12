#include "ee14lib.h"

int mpu_init(EE14Lib_Pin SCL, EE14Lib_Pin SDA, int MPU_ADDRESS) {

    unsigned char buf[2];
    i2c_init(I2C1, SCL, SDA);
    
    // Wakes the gyroscope sensor
    buf[0] = 0x6B; // PWR_MGMT_1 location
    buf[1] = 0x00; // Set all bits 0
    i2c_write(I2C1, MPU_ADDRESS, buf, 2);

    // Configure the FS_SEL to measure ±250(°/s)
    buf[0] = 0x1B;
    buf[1] = 0b00 << 3;
    i2c_write(I2C1, MPU_ADDRESS, buf, 2);
}

void mpu_read(I2C_TypeDef* I2C, EE14Lib_Pin SCL, EE14Lib_Pin SDA, int MPU_ADDRESS, int output[3]) {

    static uint8_t GYRO_ADDRESS = 0x43;
    static uint8_t reading[6];

    i2c_write(I2C, MPU_ADDRESS, &GYRO_ADDRESS, 1);
    i2c_read(I2C, MPU_ADDRESS, reading, 6);

    int16_t gx = (reading[0] << 8) | reading[1];
    int16_t gy = (reading[2] << 8) | reading[3];
    int16_t gz = (reading[4] << 8) | reading[5];

    // Convert using 131 LSB/°/s
    output[0] = gx / 131;
    output[1] = gy / 131;
    output[2] = gz / 131;

}