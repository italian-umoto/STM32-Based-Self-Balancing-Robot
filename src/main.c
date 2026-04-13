#include "ee14lib.h"
#include <stdio.h>

// IMU Pins
const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;

volatile uint32_t SysTick_Triggered = 0;
 
float actual_theta = 0;

static int GYRO_DATA[3];
static int ACCL_DATA[3];

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main() {

    host_serial_init(9600);
    SysTick_initialize();
    mpu_init(MPU_SCL, MPU_SDA);

    // uint32_t last_time = SysTick_Triggered;

    while(1) {

        gyro_read(I2C1, GYRO_DATA);
        accel_read(I2C1, ACCL_DATA);

        printf("%d, %d, %d\n", ACCL_DATA[0], ACCL_DATA[1], ACCL_DATA[2]);

        // uint32_t current_time = SysTick_Triggered;
        // float dt = (current_time - last_time) / 1000.0f;
        // last_time = current_time;

        // actual_theta += GYRO_DATA[1] * dt;

        // int whole = (int)actual_theta;
        // int frac = (int)((actual_theta - whole) * 1000);

        // if (frac < 0) frac = -frac;

        // printf("%d.%03d\n", whole, frac);

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