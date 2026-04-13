#include "ee14lib.h"
#include <stdio.h>
#include <math.h>

// IMU Pins
const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;

volatile uint32_t SysTick_Triggered = 0;

float actual_theta = 0.0f;

static int16_t RAW_GYRO_DATA;
static int16_t RAW_ACCL_DATA[2];

float alpha = 0.75f;

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main() {

    host_serial_init(9600);
    SysTick_initialize();
    mpu_init(MPU_SCL, MPU_SDA);

    const uint32_t dt_ms = 10;
    const float dt_s = dt_ms / 1000.0f;

    // Initial accelerometer angle estimate
    accel_read(I2C1, 0, 2, RAW_ACCL_DATA);   // X and Z
    float ax = RAW_ACCL_DATA[0] / 16384.0f;
    float az = RAW_ACCL_DATA[1] / 16384.0f;

    // In degrees
    actual_theta = atan2f(ax, az) * 180.0f / 3.1415926f;

    while (1) {
        uint32_t start = SysTick_Triggered;

        // Read sensors
        RAW_GYRO_DATA = gyro_read(I2C1, 1);       // Y axis gyro
        accel_read(I2C1, 0, 2, RAW_ACCL_DATA);    // X and Z accel

        // Convert gyro to deg/s
        float GYRO_OMEGA = RAW_GYRO_DATA / 131.0f;

        // Convert accel to g
        ax = RAW_ACCL_DATA[0] / 16384.0f;
        az = RAW_ACCL_DATA[1] / 16384.0f;

        // Accelerometer angle in degrees
        float ACCL_ANGLE = atan2f(ax, az) * 180.0f / 3.1415926f;

        // Complementary filter
        actual_theta = alpha * (actual_theta + GYRO_OMEGA * dt_s) + (1.0f - alpha) * ACCL_ANGLE;

        // Print without %f
        int whole = (int)actual_theta;
        int frac = (int)((actual_theta - whole) * 100);
        if (frac < 0) frac = -frac;
        printf("%d.%02d\n", whole, frac);

        while (SysTick_Triggered - start < dt_ms) {}
    }
}

// Setup 1ms delay
void SysTick_initialize(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 3999; // 1kHz Reload
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void  SysTick_Handler(void) {
    SysTick_Triggered++;
}

void delay_ms(uint32_t ms) {
    uint32_t start = SysTick_Triggered;
    while(SysTick_Triggered - start < ms);
};