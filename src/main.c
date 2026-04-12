#include "ee14lib.h"
#include <stdio.h>

const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;
const uint8_t MPU_ADDRESS = 0x68;

static 

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main() {

    host_serial_init(9600);
    mpu_init(MPU_SCL, MPU_SDA, MPU_ADDRESS);

    while(1) {

        int GYRO_DATA[3];
        mpu_read(I2C1, MPU_SCL, MPU_SDA, MPU_ADDRESS, GYRO_DATA);
        printf("%d, %d, %d\n", GYRO_DATA[0], GYRO_DATA[1], GYRO_DATA[2]);

    }

}