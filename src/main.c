#include "ee14lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// IMU Pins
const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;

EE14Lib_Pin Test = D6;

volatile uint32_t SysTick_Triggered = 0;

// PID Parameters
const float KI = 0;//0                              // 0
const float KP = -15; //-15                         // -15
const float KD = -12; //-12                       // -16.7
const float bal_theta = 0;                        // 0.6

float error = 0;
float past_error = 0;
float PWM_setpoint = 0;
float last_error = 0;

// Complementary filter gain
static float alpha = 0.95f;

// Function Prototype
float complementary_update(float old_theta, float gyro_rate, float accel_angle, float dt_s);

int main() {
    host_serial_init(9600);
    SysTick_initialize();
    mpu_init(MPU_SCL, MPU_SDA);
    motor_config();

    gpio_config_mode(Test, OUTPUT);

    // Initial angle from accelerometer
    float actual_theta = accel_angle_deg(I2C1, 1, 2);

    float dt_ms = 10;
    float dt_s = dt_ms / 1e3f;

    while (1) {
        uint32_t loop_start = SysTick_Triggered;

        gpio_write(Test,0);
        // --- Sensor reads ---
        float gyro_omega = gyro_rate_dps(I2C1, 2);
        float accl_angle = accel_angle_deg(I2C1, 1, 2);
        gpio_write(Test,1);

        actual_theta = complementary_update(actual_theta, gyro_omega, accl_angle, dt_s);

        // --- PID ---
        error = actual_theta - bal_theta;
        past_error = past_error + error*dt_s;

        PWM_setpoint = KP * error + KD * gyro_omega + KI * past_error;

        print_data_usart(actual_theta, PWM_setpoint);

        if (error > 2){
            backward(-10*PWM_setpoint+130);
        } else if(error < -2) {
            forward(10*PWM_setpoint+130);
        } 
        else {
            past_error = 0;
            stop();
        }

        while (SysTick_Triggered - loop_start < (uint32_t)(dt_ms*1e2f)) {}
    }


}

float complementary_update(float old_theta, float gyro_rate, float accel_angle, float dt_s) {
    return alpha * (old_theta + gyro_rate * dt_s)
         + (1.0f - alpha) * accel_angle;
}

// Setup 1ms delay
void SysTick_initialize(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 39; // 10us reload
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void  SysTick_Handler(void) {
    SysTick_Triggered++;
}

void delay_us(uint32_t us) {
    uint32_t cutoff = us;
    uint32_t start = SysTick_Triggered;
    while(SysTick_Triggered - start < cutoff);
};