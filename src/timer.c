#include "ee14lib.h"

// Configures a PWM Timer specified by timer to operate at a frequency freq_hz in Hz
// This controls the timer clock enable, prescalar, ARR, output enable, and timer enable registers
//  timer: constant denoting which timer to operate on (ex TIM1)
//  freq_hz: the desired frequency for the timer in hertz
// Returns:
//  EE14Lib_Err_NOT_IMPLEMENTED: The provided timer is not supported
//  EE14Lib_Err_OK: When the function successfully executes
EE14Lib_Err timer_config_pwm(TIM_TypeDef* const timer, const unsigned int freq_hz)
{
    // Enable the clock for the timer
    if(timer == TIM1){
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    else if(timer == TIM2){
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    }
    else if(timer == TIM15){
        RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    }
    else if(timer == TIM16){
        RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    }
    else{
        return EE14Lib_Err_NOT_IMPLEMENTED;
    }

    // Top-level control registers are fine with defaults (except for turning it on, later)

    // Calculate how much we need to scale down the frequency by
    float gain = (4000000)/freq_hz;

    // Set the prescalar value to the smallest possible value for maximum PWM granularity
    // Casting to an int rounds down the value. This accounts for the fact that
    //  a PSC value of n corresponds to a division by n+1
    uint16_t PS_val = (int)(gain/65536); 

    // Assign the PSC value
    timer->PSC = PS_val;

    // Set the counter reset to the value that multiplies with PS_val to get the gain.
    // The +1 and the -1 factors are to undo the zero indexing of the prescalar and
    //  to zero index the ARR register respectively
    timer->ARR = (int)(gain/(PS_val + 1) - 1);

    // Set the main output enable
    timer->BDTR |= TIM_BDTR_MOE;

    // And enable the timer itself
    timer->CR1 |= TIM_CR1_CEN;

    return EE14Lib_Err_OK;
}


// Mapping of GPIO pin to timer channel
// Lowest bit is whether this is an inverted channel (N),
// remaining bits are zero-indexed (0-3 for TIMx_1-TIMx_4)
//   1  -> 0
//   1N -> 1
//   2  -> 2
//   2N -> 3
//   3  -> 4
//   3N -> 5
//   4  -> 6
// -1 for pins which are not PWM-capable / not on this timer.
int g_Timer1Channel[D13+1] = {
  -1,-1,-1,-1,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
  -1,-1, 1,-1,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
   4, 2,-1, 3,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  -1,-1, 5,-1,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  -1, 0, 6,-1,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  -1,-1         // D12=PB4,D13=PB3.
};

int g_Timer2Channel[D13+1] = {
   0, 2, 6,-1,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
   0,-1,-1, 4,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
  -1,-1,-1,-1,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  -1,-1,-1,-1,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  -1,-1,-1,-1,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  -1, 2         // D12=PB4,D13=PB3.
};

int g_Timer15Channel[D13+1] = {
  -1, 1, 2,-1,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
  -1,-1,-1, 0,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
  -1,-1,-1,-1,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  -1,-1,-1,-1,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  -1,-1,-1,-1,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  -1,-1         // D12=PB4,D13=PB3.
};

int g_Timer16Channel[D13+1] = {
  -1,-1,-1,-1,  // A0=PA0,A1=PA1,A2=PA3,A3=PA4
  -1, 0,-1,-1,  // A4=PA5,A5=PA6,A6=PA7,A7=PA2
  -1,-1,-1,-1,  // D0=PA10,D1=PA9,D2=PA12,D3=PB0
  -1, 1,-1,-1,  // D4=PB7,D5=PB6,D6=PB1,D7=PC14
  -1,-1,-1,-1,  // D8=PC15,D9=PA8,D10=PA11,D11=PB5
  -1,-1         // D12=PB4,D13=PB3.
};


// Configure the PWM of a PWM capable pin
//  timer: constant denoting which PWM timer to use
//  pin: Label on the Nucleo board for the pin to use (ex D9)
//  duty: duty cycle of the output. (0-1023)  0 = always low, 1023 = always high
// Returns:
//  EE14Lib_ERR_INVALID_CONFIG: When the provided pin is not attached to the timer
//  EE14Lib_Err_OK: When the function successfully executes
EE14Lib_Err timer_config_channel_pwm(TIM_TypeDef* const timer, const EE14Lib_Pin pin, const unsigned int duty)
{
    int channel = -1;
    if(timer == TIM1){
        channel = g_Timer1Channel[pin];
    } else if(timer  == TIM2){
        channel = g_Timer2Channel[pin];
    } else if(timer == TIM15){
        channel = g_Timer15Channel[pin];
    } else if(timer == TIM16){
        channel = g_Timer16Channel[pin];
    }

    if(channel < 0){
        return EE14Lib_ERR_INVALID_CONFIG;
    }

    int channel_idx = channel >> 1; // Lowest bit is N


    
    // The max counter value determines the compare for 100% duty cycle
    // Other duty cycles are found from taking fractions of this value
    uint16_t arr_value = timer->ARR;

    // Set the compare value. Duty values from 0-1023 so the duty cycle = duty/1023 * 100%
    // This makes the compare value = duty cycle % * arr value
    *((unsigned int*)timer + 13 + channel_idx) = (int)(duty * arr_value / 1023);


    // Enable PWM mode, and set preload enable (only update counter on rollover)
    if(channel_idx == 0){
        timer->CCMR1 &= ~(TIM_CCMR1_OC1M);
        timer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
        timer->CCMR1 |= TIM_CCMR1_OC1PE;
    } else if(channel_idx == 1){
        timer->CCMR1 &= ~(TIM_CCMR1_OC2M);
        timer->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
        timer->CCMR1 |= TIM_CCMR1_OC2PE;
    } else if(channel_idx == 2){
        timer->CCMR2 &= ~(TIM_CCMR2_OC3M);
        timer->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
        timer->CCMR2 |= TIM_CCMR2_OC3PE;
    } else { // Must be 3
        timer->CCMR2 &= ~(TIM_CCMR2_OC4M);
        timer->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
        timer->CCMR2 |= TIM_CCMR2_OC4PE;
    }

    // Enable the capture/compare output
    timer->CCER |= 1 << (2*channel); // Primary enables are 0, 4, 8, 12; inverted are 2, 6

    if(timer == TIM1 || timer == TIM2){
        gpio_config_alternate_function(pin, 1); // AFR = 1 is timer mode for timers 1 & 2
    } else {
        gpio_config_alternate_function(pin, 14); // AFR = 14 for timers 15 & 16
    }

    return EE14Lib_Err_OK;
}

