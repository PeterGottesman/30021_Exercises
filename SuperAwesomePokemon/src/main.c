/*
**
**                           Main.c
**
**
**********************************************************************/
/*
  Last committed:     $Revision: 00 $
  Last changed by:    $Author: $
  Last changed date:  $Date:  $
  ID:                 $Id:  $

**********************************************************************/
#include "stm32f30x_conf.h"
#include "30021_io.h"
#include "gpio.h"
#include <stdio.h>

#define Ftimer 64000000
#define Fck 2560000

int initTimer(void)
{
    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    // Disable and set to defaults
    TIM16->CR1 = 0x0;

    // Reload after 255 cycles
    TIM16->ARR = 255;

    // Set duty cycle to 50%
    TIM16->CCR1 = 128;

    // PSC = (Ftimer/Fck) - 1
    // Ftimer = 64MHz
    // Fck = 2.56MHz
    TIM16->PSC = (Ftimer/Fck)-1;

    // Configure channel 4 as output PWM mode 1 w/ preload enable
    MODIFY_REG(TIM16->CCMR1, TIM_CCMR1_CC1S | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M,
	       TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

    MODIFY_REG(TIM16->BDTR, TIM_BDTR_MOE | TIM_BDTR_OSSI | TIM_BDTR_OSSR, TIM_BDTR_MOE);

    // Set output enable and polarity
    MODIFY_REG(TIM16->CCER, TIM_CCER_CC1P | TIM_CCER_CC1E, TIM_CCER_CC1E);

    // Enable
    TIM16->CR1 |= 0x1;

    // Disable interrupt
    TIM16->DIER &= ~0x0001;

    return 0;
}

int main(void)
{
    initTimer();
    initPinAlternate(GPIOA, GPIO_PinSource6, GPIO_AF_1);

    while (1){}
}
