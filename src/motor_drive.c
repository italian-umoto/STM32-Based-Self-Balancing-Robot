//* motor_drive.c
#include "ee14lib.h"

#define FORWARD_LEFT D13  // 
#define FORWARD_RIGHT A1  // 
#define BACKWARD_LEFT A4  // Red
#define BACKWARD_RIGHT A2 // Brown

#define PWM_MAX 1023

static int clamp_pwm(int speed) {
    if (speed < 0) {
        return 0;
    }
    if (speed > PWM_MAX) {
        return PWM_MAX;
    }
    return speed;
}

EE14Lib_Err motor_config() {
    gpio_config_alternate_function(FORWARD_LEFT, 1);
    gpio_config_alternate_function(FORWARD_RIGHT, 1);
    gpio_config_alternate_function(BACKWARD_LEFT, 1);
    gpio_config_alternate_function(BACKWARD_RIGHT, 1);

    timer_config_pwm(TIM2, 300);

    stop();

    return EE14Lib_Err_OK;
}

EE14Lib_Err forward(int speed) {
    speed = clamp_pwm(speed);

    timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
    timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0); 

    timer_config_channel_pwm(TIM2, FORWARD_LEFT, speed);  
    timer_config_channel_pwm(TIM2, FORWARD_RIGHT, speed); 

    return EE14Lib_Err_OK;
}

EE14Lib_Err backward(int speed) {
    speed = clamp_pwm(speed);

    timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0); 
    timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0); 

    timer_config_channel_pwm(TIM2, BACKWARD_LEFT, speed);  
    timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, speed); 

    return EE14Lib_Err_OK;
}

EE14Lib_Err stop() {
    timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0);  
    timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0); 
    timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0); 
    timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0); 

    return EE14Lib_Err_OK;
}

EE14Lib_Err turn_left(int speed) {
    speed = clamp_pwm(speed);

    timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0);  
    timer_config_channel_pwm(TIM2, FORWARD_RIGHT, speed); 
    timer_config_channel_pwm(TIM2, BACKWARD_LEFT, speed); 
    timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0);  

    return EE14Lib_Err_OK;
}

EE14Lib_Err turn_right(int speed) {
    speed = clamp_pwm(speed);

    timer_config_channel_pwm(TIM2, FORWARD_LEFT, speed);  
    timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0); 
    timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
    timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, speed);  

    return EE14Lib_Err_OK;
}

/* move
General linear movement and rotation
linear_velocity: -1023 = backwards, 1023 = forwards
angular_velocity: positive/negative rotation command
*/
EE14Lib_Err move(int16_t linear_velocity, int16_t angular_velocity) {
    int8_t left_dir = 0; 
    int8_t right_dir = 0; 
   
    int32_t F1 = (linear_velocity + angular_velocity) >> 1;
    int32_t F2 = (linear_velocity - angular_velocity) >> 1;

    if (F1 < 0) {
        left_dir = 1;
        F1 = -F1;
    }

    if (F2 < 0) {
        right_dir = 1;
        F2 = -F2;
    }

    if (F1 > 1023 || F2 > 1023) {
        if (F1 > F2) {
            F2 = (F2 * 1023) / F1;
            F1 = 1023;
        } else {
            F1 = (F1 * 1023) / F2;
            F2 = 1023;
        }
    }

    F1 = clamp_pwm(F1);
    F2 = clamp_pwm(F2);

    if (left_dir == 0) {
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, F1);  
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, 0); 
    } else {
        timer_config_channel_pwm(TIM2, FORWARD_LEFT, 0);  
        timer_config_channel_pwm(TIM2, BACKWARD_LEFT, F1); 
    }

    if (right_dir == 0) {
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, F2);  
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, 0); 
    } else {
        timer_config_channel_pwm(TIM2, FORWARD_RIGHT, 0);  
        timer_config_channel_pwm(TIM2, BACKWARD_RIGHT, F2); 
    }

    return EE14Lib_Err_OK;
}