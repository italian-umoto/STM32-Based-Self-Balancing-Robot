//* Servo_funcs.c: Some simple functions to easily initialize and use servo motors
#include "ee14lib.h"

// Initialize a servo connected to the provided timer (ex TIM1) and pin (ex D9)
// Inputs:
//  timer: Constant representing the timer to use (ex TIM1)
//  pin: Silkscreen of the pin on the Nucleo to use (ex D9)
// Returns EE14Lib_Err_OK on success
EE14Lib_Err configure_servo(TIM_TypeDef* const timer, const EE14Lib_Pin pin){
    gpio_config_alternate_function(pin, 1);
    timer_config_pwm(timer, 50);
    return EE14Lib_Err_OK;
}


// Set the position of a servo to a position in degrees
// Inputs:
//  timer: Constant representing the timer to use (ex TIM1)
//  pin: Silkscreen of the pin on the Nucleo to use (ex D9)
//  degrees: The position of the servo to move to in degrees. Range is [0, 90]
// Returns EE14Lib_ERR_INVALID_CONFIG when position is out of range
// Returns EE14Lib_Err_OK on success
EE14Lib_Err set_servo(TIM_TypeDef* const timer, const EE14Lib_Pin pin, float degrees){
    
    uint16_t base = 60; // Minimum PWM is 5%
    uint16_t shift = degrees * 10 / 90; // Maximum PWM is 10% at 90 degrees

    // Ensure we are moving to a valid position
    if(degrees < 0 || degrees > 90){
        return EE14Lib_ERR_INVALID_CONFIG;
        printf("Failing\n");
    } else {
        // If valid, change the PWM to move the servo
        timer_config_channel_pwm(timer, pin, base + shift);  
    }

    return EE14Lib_Err_OK;
}