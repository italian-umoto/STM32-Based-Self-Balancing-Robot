// main.c
// Control the position of a servo motor via PWM

/*
#include "ee14lib.h"

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main(){
    // Set pin and timer
    host_serial_init(9600);
    EE14Lib_Pin motor_pin = D6;
    TIM_TypeDef* motor_timer = TIM1; 

    // Configure the Servo
    configure_servo(motor_timer, motor_pin);

    // Initialize the servo to a known position
    volatile int degrees = 45;
    set_servo(motor_timer, motor_pin, degrees);

    int mode = 0;

    // Sweep servo forward then quickly bring it back
    while(1){
        //Limit the speed of the servo
        for(volatile int i = 0; i < 15000; i++){}

        // Logic for the next position to go to
        if(mode == 0){
            if(degrees < 85){
                degrees = degrees + 5;
            } else {
                mode = 1;
            }
        } else if (mode == 1){
            if (degrees > 5){
                degrees = degrees - 5;
            } else {
                mode = 0;
            }

        }
        // Adjust the PWM signal
       // printf("%d\n", degrees);
        uint16_t base = 60; // Minimum PWM is 5%
        uint16_t shift = degrees * 10 / 90; // Maximum PWM is 10% at 90 degrees
        printf("%d, %d\n", degrees, base + shift);

        set_servo(motor_timer, motor_pin, degrees);
    }
}
*/