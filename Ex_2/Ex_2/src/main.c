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
#include "string.h"
#include "lcd.h"
#include "flash.h"
#include "gpio.h"
#include "stdio.h"

#define FULL_SCALE 4095
#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))   //calibrated at 3.3V@ 30C 

volatile int display=0;
volatile int calculate_calfact=0;

void initADC(int calib)
{
    RCC->CFGR2 &= ~RCC_CFGR2_ADCPRE12; // Clear ADC12 prescaler bits
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV8; // Set ADC12 prescaler to 8
    RCC->AHBENR |= RCC_AHBPeriph_ADC12; // Enable clock for ADC12

    ADC1->CR = 0x00000000; // Clear CR register
    ADC1->CFGR &= 0xFDFFC007; // Clear ADC1 config register
    ADC1->SQR1 &= ~ADC_SQR1_L; // Clear regular sequence register 1

    //Verify ADVREG state (ADC voltage regulator)
    //1. Set ADVREGEN from '10' (disabled state) to '00' and then to '01' (enabled)
    //2. Wait 10uS (worst case) before performing calibration and/or
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    //Wait for at least 10uS before continuing...
    for(uint32_t i = 0; i < 10000; i++);
    if(calib)
    {
        ADC1->CR &= ~ADC_CR_ADEN; //Make sure ADC is disabled
        ADC1->CR &= ~ADC_CR_ADCALDIF; //Use single ended calibration
        ADC1->CR |= ADC_CR_ADCAL; //Start calibration
        while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL){} //Wait for calibration to finish
        //Wait > 4 ADC clock cycles after ADCAL bit is cleared
        for(uint32_t i = 0; i < 100; i++);
    }
    //Enable ADC peripheral
    ADC1->CR |= ADC_CR_ADEN;
    //wait for ADC1 to be ready to start conversion
    while (!ADC1->ISR & ADC_ISR_ADRD){}

}

uint16_t ADC_measure_PA(uint8_t ch)
{
    ADC_RegularChannelConfig(ADC1,ch, 1, ADC_SampleTime_1Cycles5);
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    uint16_t x = ADC_GetConversionValue(ADC1); // Read the ADC value
    return x;
}

void TIM2_IRQHandler(void)
{
    display=1;
    TIM2->SR &= ~0x1; //Clear Interrupt bit
}

int initTimer(void)
{
    // Initialize Syscfg clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Disable and set to defaults
    TIM2->CR1 = 0x0;

    TIM2->ARR = 6300000; // Computed from formula on handout
    TIM2->PSC = 0x0;

    // Enable
    TIM2->CR1 |= 0x1;
    TIM2->DIER |= 0x0001;

    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM2_IRQn);

    return 0;
}

void EXTI9_5_IRQHandler(void)
{
    calculate_calfact=1;
    EXTI->PR |= EXTI_PR_PR0;
}

void initINT9_5(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG->EXTICR[1] &= ~(0xF << 4);
    SYSCFG->EXTICR[1] |= 1 << 4;
    
    EXTI->RTSR |= 1 << 5;
    EXTI->FTSR &= ~(1 << 5);

    EXTI->IMR |= 1 << 5;

    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void save_cal2flash(float calfactor1, float calfactor2)
{
    FLASH_Unlock();

    write_float_flash(PG31_BASE, 0, calfactor1);
    write_float_flash(PG31_BASE, 1, calfactor2);
    
    FLASH_Lock();
}

int main(void)
{
    float Vdda, calfactor1, calfactor2;
    char meas1[26];
    char meas2[26];
    uint8_t fbuffer[512];

    initPin(GPIOA,0,PIN_MODE_INPUT,PIN_PUPD_NONE, PIN_OTYPE_RESET);
    initPin(GPIOA,1,PIN_MODE_INPUT,PIN_PUPD_NONE, PIN_OTYPE_RESET);
    init_spi_lcd();
    memset(fbuffer,0x00,512);
    lcd_push_buffer(fbuffer);

    initADC(1);
    initTimer();
    initJoystick();
    initINT9_5();

    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    uint16_t Vref_data = ADC_GetConversionValue(ADC1); // Read the ADC value
    Vdda = 3.3 * VREFINT_CAL/Vref_data;

    // Save the two calfactors to flash only once
    //save_cal2flash(0.998, 0.997); 

    while(1)
    {
        if(display==1)
        {
	    uint16_t CH1 = ADC_measure_PA(ADC_Channel_1);
	    uint16_t CH2 = ADC_measure_PA(ADC_Channel_2);
	    float Vc1 = Vdda/FULL_SCALE * CH1 * calfactor1;
	    float Vc2 = Vdda/FULL_SCALE * CH2 * calfactor2;
	    snprintf(meas1,25,"%.2fV, cal: %f", Vc1, calfactor1);
            snprintf(meas2,25,"%.2fV, cal: %f", Vc2, calfactor2);
            lcd_write_string("OUTPUT1",fbuffer,0,0);
            lcd_write_string(meas1,fbuffer,3,1);
            lcd_write_string("OUTPUT2",fbuffer,0,2);
            lcd_write_string(meas2,fbuffer,3,3);
            lcd_push_buffer(fbuffer);
            display=0;
        }

	if(calculate_calfact==1)
	{

	    //Read calfactors from flash
	    calfactor1 = read_float_flash(PG31_BASE, 0);
	    calfactor2 = read_float_flash(PG31_BASE, 1);
    
	    if(calfactor1 < 0.5 || calfactor1 > 1.5)
	    {
		calfactor1 = 1.0;
	    }
	    if(calfactor2 < 0.5 || calfactor2 > 1.5)
	    {
		calfactor2 = 1.0;
	    }

	    //Calibrate the calfactors by sending through 3.2V and clicking
	    //...joystick center button
	    
	    /*
	    uint32_t sum = 0;
	    uint16_t avg = 0;
	    float V1, V2;
	    for (int i = 0; i < 16; ++i)
	    {
		sum += ADC_measure_PA(ADC_Channel_1);
	    }
	    avg = (uint16_t)(sum/16);

	    V1 = Vdda/FULL_SCALE * avg;
	    calfact1 = 3.2/V1;
	    
	    sum = 0;
	    avg = 0;
	    for (int i = 0; i < 16; ++i)
	    {
		sum += ADC_measure_PA(ADC_Channel_2);
	    }
	    avg = (uint16_t)(sum/16);

	    V2 = Vdda/FULL_SCALE * avg;
	    calfact2 = 3.2/V2;
	    calculate_calfact=0;
	    */	
	}

    }
}
