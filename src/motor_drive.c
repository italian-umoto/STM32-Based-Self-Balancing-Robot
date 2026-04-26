//* Servo_funcs.c: Some simple functions to easily initialize and use servo motors
#include "ee14lib.h"

// Initialize a servo connected to the provided timer (ex TIM1) and pin (ex D9)
// Inputs:
//  timer: Constant representing the timer to use (ex TIM1)
//  pin: Silkscreen of the pin on the Nucleo to use (ex D9)
// Returns EE14Lib_Err_OK on success

#define FORWARD_LEFT A1
#define FORWARD_RIGHT A7
#define BACKWARD_LEFT A2
#define BACKWARD_RIGHT A4


EE14Lib_Err motor_config(){
    gpio_config_alternate_function(FORWARD_LEFT, 1);
    gpio_config_alternate_function(FORWARD_RIGHT, 1);
    gpio_config_alternate_function(BACKWARD_LEFT, 1);
    gpio_config_alternate_function(BACKWARD_RIGHT, 1);
    timer_config_pwm(TIM2, 50);
    return EE14Lib_Err_OK;
}

EE14Lib_Err forward(int speed){
    // Ensure we are moving to a valid position
    if(speed >= 300 && speed < 1024){
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, speed);  
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, speed); 
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0); 
    }
    return EE14Lib_Err_OK;
}

EE14Lib_Err backward(int speed){
    // Ensure we are moving to a valid position
    if(speed >= 300 && speed < 1024){
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, speed);  
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, speed); 
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0); 
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0); 
    }
    return EE14Lib_Err_OK;
}

EE14Lib_Err turn_left(int speed){
    // Ensure we are moving to a valid position
    if(speed >= 300 && speed < 1024){
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0);  
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, speed); 
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, speed); 
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0);  
    }
    return EE14Lib_Err_OK;
}

EE14Lib_Err turn_right(int speed){
    // Ensure we are moving to a valid position
    if(speed >= 300 && speed < 1024){
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, speed);  
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0); 
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, speed);  
    }
    return EE14Lib_Err_OK;
}

/* move
General linear movement and rotation
linear_velocity: The forwards linear velocity of the car. -1023 = backwards, 1023 = forwards
angular_velocity: The CCW rotation. +1023 = max right, -1023 = max left
If the robot cannot accomodate this, it will go at the specified angle of curvature at max speed
*/
EE14Lib_Err move(int16_t linear_velocity, int16_t angular_velocity){
    int8_t left_dir = 0; // 0 = Forwards, 1 = Backwards
    int8_t right_dir = 0; // 0 = Forwards, 1 = Backwards
   
    int32_t F1 = (linear_velocity + angular_velocity) >> 1;
    int32_t F2 = (linear_velocity - angular_velocity) >> 1;

    if(F1 < 0){
        left_dir = 1;
        F1 = -1 * F1;
    }
    if(F2 < 0){
        right_dir = 1;
        F2 = -1 * F2;
    }

    // Scale out of bound forces properly
    if(F1 > 1023 || F2 > 1023){
        if(F1 > F2){
            F2 = (F2 << 10) / F1;
            F1 = 1023;
        } else {
            F1 = (F1 << 10) / F2;
            F2 = 1023;
        }
    }

    if(left_dir == 0){
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, F1);  
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
    } else {
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0);  
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, F1); 
    }

    if(right_dir == 0){
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, F2);  
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0); 
    } else {
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0);  
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, F2); 
    }
}

