#include "ee14lib.h"
#include <stdio.h>
#include <math.h>

// IMU Pins
const EE14Lib_Pin MPU_SCL = D1;
const EE14Lib_Pin MPU_SDA = D0;

// PID Parameters
const float KI = -1;
const float KP = -1;
const float bal_theta = 0; // Change to natural balance point

float error = 0;
float past_error = 0;
float PWM_setpoint = 0;

// Complementary filter gain
static float alpha = 0.75f;

// Function Prototype
float complementary_update(float old_theta, float gyro_rate, float accel_angle, float dt_s);

int main() {
    host_serial_init(9600);
    SysTick_initialize();
    mpu_init(MPU_SCL, MPU_SDA);

    const uint32_t dt_ms = 10;
    const float dt_s = dt_ms / 1000.0f;

    // Initial angle from accelerometer
    float actual_theta = accel_angle_deg(I2C1, 0, 2);

    while (1) {
        float gyro_omega = gyro_rate_dps(I2C1, 2);
        float accl_angle = accel_angle_deg(I2C1, 1, 2);

        actual_theta = complementary_update(actual_theta, gyro_omega, accl_angle, dt_s);

        ////////////////

        error = actual_theta - bal_theta; 
        past_error = past_error + error;

        PWM_setpoint = KP*error + KI*past_error;

        print_data_usart(actual_theta, PWM_setpoint);


        ////////////////

        delay_ms(dt_ms);
    }
}

// One complementary-filter update step
float complementary_update(float old_theta, float gyro_rate, float accel_angle, float dt_s) {
    return alpha * (old_theta + gyro_rate * dt_s)
         + (1.0f - alpha) * accel_angle;
}