#include "ee14lib.h"

static int gyro_offset[3] = {0, 0, 0};
static uint8_t GYRO_ADDRESS = 0x43;
static uint8_t MPU_ADDRESS = 0x68;

void mpu_init(EE14Lib_Pin SCL, EE14Lib_Pin SDA) {

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

    gyro_calibrate(500);

}

void gyro_read(I2C_TypeDef* I2C, int output[3]) {

    static uint8_t reading[6];

    i2c_write(I2C, MPU_ADDRESS, &GYRO_ADDRESS, 1);
    i2c_read(I2C, MPU_ADDRESS, reading, 6);

    int16_t gx = (reading[0] << 8) | reading[1];
    int16_t gy = (reading[2] << 8) | reading[3];
    int16_t gz = (reading[4] << 8) | reading[5];

    // Subtract calibration offsets
    gx -= gyro_offset[0];
    gy -= gyro_offset[1];
    gz -= gyro_offset[2];

    // Convert using 131 LSB/°/s
    output[0] = gx / 131;
    output[1] = gy / 131;
    output[2] = gz / 131;
}

void gyro_calibrate(const int samples) {
    long sum_x = 0;
    long sum_y = 0;
    long sum_z = 0;

    for (int k = 0; k < samples; k++) {
        uint8_t reading[6];

        i2c_write(I2C1, MPU_ADDRESS, &GYRO_ADDRESS, 1);
        i2c_read(I2C1, MPU_ADDRESS, reading, 6);

        int16_t gx = (reading[0] << 8) | reading[1];
        int16_t gy = (reading[2] << 8) | reading[3];
        int16_t gz = (reading[4] << 8) | reading[5];

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

    }

    gyro_offset[0] = sum_x / samples;
    gyro_offset[1] = sum_y / samples;
    gyro_offset[2] = sum_z / samples;
}

void accel_read(I2C_TypeDef* I2C, int output[3]) {
    uint8_t ACCEL_ADDRESS = 0x3B;
    uint8_t reading[6];

    i2c_write(I2C, MPU_ADDRESS, &ACCEL_ADDRESS, 1);
    i2c_read(I2C, MPU_ADDRESS, reading, 6);

    int16_t ax = (reading[0] << 8) | reading[1];
    int16_t ay = (reading[2] << 8) | reading[3];
    int16_t az = (reading[4] << 8) | reading[5];

    output[0] = ax;
    output[1] = ay;
    output[2] = az;
}