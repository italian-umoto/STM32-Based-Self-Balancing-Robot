// main.c
// Control the position of a servo motor via PWM

#include "ee14lib.h"

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main(){
    // Set pin and timer
    host_serial_init(9600);

    // Configure the Servo
    motor_config();

    while(1){
        forward(700);   
    }


}
