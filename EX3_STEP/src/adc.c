#include "adc.h"

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
