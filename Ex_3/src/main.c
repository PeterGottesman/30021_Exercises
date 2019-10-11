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
#include "lcd.h"
#include "adc.h"
#include <string.h>
#include <stdio.h>

#define Ftimer 64000000
#define Fck 2560000
#define FULL_SCALE 4095
#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))   //calibrated at 3.3V@ 30C

#define TARGET_VOLTAGE 1.0f
#define EPSILON 0.05

volatile int display = 1; 

int initTimer(uint8_t duty_clocks)
{
    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    // Disable and set to defaults
    TIM16->CR1 = 0x0;

    // Reload after 255 cycles
    TIM16->ARR = 255;

    // Set duty cycle to 50%
    TIM16->CCR1 = duty_clocks;

    // PSC = (Ftimer/Fck) - 1
    // Ftimer = 64MHz
    // Fck = 2.56MHz
    TIM16->PSC = (Ftimer/Fck)-1;

    // Configure channel 4 as output PWM mode 1 w/ preload enable
    MODIFY_REG(TIM16->CCMR1, TIM_CCMR1_CC1S | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M,
	       TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

    MODIFY_REG(TIM16->BDTR, TIM_BDTR_MOE | TIM_BDTR_OSSI | TIM_BDTR_OSSR, TIM_BDTR_MOE);

    // Set output enable and polarity
    MODIFY_REG(TIM16->CCER, TIM_CCER_CC1P  | TIM_CCER_CC1E, TIM_CCER_CC1E);

    // Enable
    TIM16->CR1 |= 0x1;

    // Disable interrupt
    TIM16->DIER &= ~0x0001;

    return 0;
}

void TIM2_IRQHandler(void)
{
    display=1;
    TIM2->SR &= ~0x1; //Clear Interrupt bit
}

int initDisplayTimer(void)
{
    // Initialize Syscfg clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Disable and set to defaults
    TIM2->CR1 = 0x0;

    TIM2->ARR = 63000000; // Computed from formula on handout
    TIM2->PSC = 0x00;

    // Enable
    TIM2->CR1 |= 0x1;
    TIM2->DIER |= 0x0001;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    return 0;
}


int main(void)
{
    float Vdda, duty;
    uint16_t Vref_data, duty_clocks;

    char meas1[26];
    uint8_t fbuffer[512];

    duty = .2f;
    duty_clocks = (duty * 255.0);


    initPin(GPIOB, 1, PIN_MODE_INPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);
    init_spi_lcd();
    memset(fbuffer,0x00,512);
    lcd_push_buffer(fbuffer);

    initADC(1);

    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    Vref_data = ADC_GetConversionValue(ADC1); // Read the ADC value
    Vdda = 3.3 * VREFINT_CAL/Vref_data;

    initPinAlternate(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    initTimer(duty_clocks);
    initDisplayTimer();

    while(1)
    {
	float Vc1;
	uint32_t sum = 0;
	uint16_t avg = 0;

	for (int i = 0; i < 16; ++i)
	{
	    sum += ADC_measure_PA(ADC_Channel_12);
	}
	avg = (uint16_t)(sum/16.0);

	Vc1 = Vdda/FULL_SCALE * avg;

	if (duty_clocks > 0 && duty_clocks < 255)
	{
	    if (Vc1 + EPSILON < TARGET_VOLTAGE)
	    {
		duty_clocks += 1;
	    } else if (Vc1 - EPSILON > TARGET_VOLTAGE)
	    {
		duty_clocks -= 1;
	    }

	    initTimer(duty_clocks);
	}
	duty = duty_clocks/255.0;

	if (display == 1)
	{
	    snprintf(meas1,25,"%.2fV Duty: %.02f%%", Vc1, duty*100);
	    lcd_write_string(meas1,fbuffer,0,0);
	    lcd_push_buffer(fbuffer);
	    display = 0;
	}
    }
}
