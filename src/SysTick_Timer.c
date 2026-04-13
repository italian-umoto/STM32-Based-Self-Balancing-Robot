#include "ee14lib.h"

volatile uint32_t SysTick_Triggered = 0;

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