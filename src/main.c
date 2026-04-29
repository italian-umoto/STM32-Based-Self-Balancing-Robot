#include "ee14lib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// IMU Pins
const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;

EE14Lib_Pin Test = D6;

volatile uint32_t SysTick_Triggered = 0;

// Motor limits
#define PWM_MIN 130
#define PWM_MAX 1023
#define ERROR_DEADBAND 1.0f

// PID Parameters
const float KI = -55;//-10
const float KP = -350;//-200 
const float KD = -40;//-25
const float bal_theta = -0.3;

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

    // gpio_config_mode(Test, OUTPUT);

    // Initial angle from accelerometer
    float actual_theta = accel_angle_deg(I2C1, 1, 2);

    // Snapshot the tick counter before entering the loop
    uint32_t last_tick = SysTick_Triggered;

    while (1) {

        // gpio_write(Test, 0);

        // --- Sensor reads ---
        float gyro_omega = gyro_rate_dps(I2C1, 0);
        float accl_angle = accel_angle_deg(I2C1, 1, 2);

        // gpio_write(Test, 1);

        // Measure dt as time between this sample and the last
        uint32_t current_tick = SysTick_Triggered;
        float dt_s = (current_tick - last_tick) * 1e-5f;
        last_tick = current_tick;

        actual_theta = complementary_update(actual_theta, gyro_omega, accl_angle, dt_s);

        // --- PID ---
        error = actual_theta - bal_theta;
        past_error = past_error + error * dt_s;
        PWM_setpoint = KP * error + KD * gyro_omega + KI * past_error;

        // print_data_usart(actual_theta, PWM_setpoint);

        // --- Motor command mapping ---
        if (fabs(error) < ERROR_DEADBAND) {
            // If we are in a region that is slightly off the balancing point, stop
            past_error = 0;
            stop();
        } 
        else {
            // Take the absolute value of PWM_setpoint for PWM magnitude
            int duty = (int)fabs(PWM_setpoint); 

            // Clamp to maximum PWM
            if (duty > PWM_MAX) {
                duty = PWM_MAX;
            }

            // Compensate for motor dead zone
            if (duty < PWM_MIN) {
                duty = PWM_MIN;
            }

            // Use PID sign to decide direction
            if (PWM_setpoint > 0) {
                forward(duty+300);
            } 
            else {
                backward(duty+300);
            }
        }

        last_error = error;
    }
}

float complementary_update(float old_theta, float gyro_rate, float accel_angle, float dt_s) {
    return alpha * (old_theta + gyro_rate * dt_s) + (1.0f - alpha) * accel_angle;
}

// Setup 10us delay
void SysTick_initialize(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 39; // 10us reload
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    SysTick_Triggered++;
}

void delay_us(uint32_t us) {
    uint32_t cutoff = us / 10;
    uint32_t start = SysTick_Triggered;
    while (SysTick_Triggered - start < cutoff);
}