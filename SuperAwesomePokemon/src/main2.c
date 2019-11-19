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
#include "adc.h"
#include <stdio.h>

#define FULL_SCALE 4095
//#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))   //calibrated at 3.3V@ 30C

int main(void)
{
    float Vdda;
    /*char meas1[26];
    char meas2[26];*/

    initPin(GPIOA,0,PIN_MODE_INPUT,PIN_PUPD_NONE, PIN_OTYPE_RESET);
    initPin(GPIOA,1,PIN_MODE_INPUT,PIN_PUPD_NONE, PIN_OTYPE_RESET);

    initADC(1);
    init_usb_uart(9600);

    ADC1_2->CCR |= ADC12_CCR_VREFEN;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_181Cycles5);
    ADC_StartConversion(ADC1); // Start ADC read
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0); // Wait for ADC read
    //uint16_t Vref_data = ADC_GetConversionValue(ADC1); // Read the ADC value
    //Vdda = 3.3 * VREFINT_CAL/Vref_data;
    Vdda = 5.0;
    float rat = Vdda/FULL_SCALE;
    printf("Ratio: %d", (int)rat);
    for(int i = 0; i < 10000000; i++);

    while(1)
    {
        uint16_t CH1=ADC_measure_PA(ADC_Channel_1);
        uint16_t CH2=ADC_measure_PA(ADC_Channel_2);
        float Vc1 = (rat) * CH1;
        float Vc2 = (rat) * CH2;
        /*snprintf(meas1,25,"%d %.2fV",CH1, Vc1);
        snprintf(meas2,25,"%d %.2fV",CH2, Vc2);
        printf("meas1: %s\tmeas2: %s\t", meas1, meas2);*/
        printf("CH1: %d\t CH2: %d\t", CH1, CH2);
        printf("Vc1: %d\t Vc2: %d\n", (int)Vc1, (int)Vc2);
        for(int i = 0; i < 1000000; i++);
    }
}
