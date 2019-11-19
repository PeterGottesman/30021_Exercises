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
#include "accel.h"
#include "spi.h"


#define Ftimer 64000000
#define Fck 2560000
#define FULL_SCALE 4095
//#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))   //calibrated at 3.3V@ 30C

int initCoilRight(void)
{
    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    // Disable and set to defaults
    TIM16->CR1 = 0x0;

    // Reload after 255 cycles
    TIM16->ARR = 1000;

    // Set duty cycle to 50%
    TIM16->CCR1 = 0;

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

    // Set complementary output enable and polarity
    MODIFY_REG(TIM16->CCER, TIM_CCER_CC1NP | TIM_CCER_CC1NE, TIM_CCER_CC1NE);

    // Enable
    TIM16->CR1 |= 0x1;

    // Disable interrupt
    TIM16->DIER &= ~0x0001;

    return 0;

}

int initCoilLeft(void)
{
    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

    // Disable and set to defaults
    TIM17->CR1 = 0x0;

    // Reload after 255 cycles
    TIM17->ARR = 1000;

    // Set duty cycle to 50%
    TIM17->CCR1 = 500;

    // PSC = (Ftimer/Fck) - 1
    // Ftimer = 64MHz
    // Fck = 2.56MHz
    TIM17->PSC = (Ftimer/Fck)-1;

    // Configure channel 4 as output PWM mode 1 w/ preload enable
    MODIFY_REG(TIM17->CCMR1, TIM_CCMR1_CC1S | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M,
	       TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

    MODIFY_REG(TIM17->BDTR, TIM_BDTR_MOE | TIM_BDTR_OSSI | TIM_BDTR_OSSR, TIM_BDTR_MOE);

    // Set output enable and polarity
    MODIFY_REG(TIM17->CCER, TIM_CCER_CC1P | TIM_CCER_CC1E, TIM_CCER_CC1E);

    // Enable
    TIM17->CR1 |= 0x1;

    // Disable interrupt
    TIM17->DIER &= ~0x0001;

    return 0;
}


int main(void)
{
    /* Electromagnet dynamic range is (roughly) from .94 to 1.23 volts */
    initCoilRight();
    initCoilLeft();
    initPinAlternate(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    initPinAlternate(GPIOB, GPIO_PinSource5, GPIO_AF_10);

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

    float x,y,z;
    uint8_t id, status;
    spi3_init();
    init_accel();
    id = accel_whoami();

    x = y = z = 0;

    float error, integral, desired, actual, output;
    float Kp, Ki, bias;
    float dt;

    dt = 1;
    Kp = 5;
    Ki = 1;
    bias = integral = 0;
    
    int count = 0;

    while (1)
    {
	/* for (int i = 0; i < 10000000; ++i); */
	count++;
        if (count % 30 == 0)
	{
          int tmp = TIM17->CCR1;
          TIM17->CCR1 = TIM16->CCR1;
          TIM16->CCR1 = tmp;
        } 

        /* uint16_t CH1=ADC_measure_PA(ADC_Channel_1); */
	/* uint16_t CH2=ADC_measure_PA(ADC_Channel_2); */
	/* float Vc1 = (rat) * CH1; */
	/* float Vc2 = (rat) * CH2; */
	/* /\*snprintf(meas1,25,"%d %.2fV",CH1, Vc1); */
	/*   snprintf(meas2,25,"%d %.2fV",CH2, Vc2); */
	/*   printf("meas1: %s\tmeas2: %s\t", meas1, meas2);*\/ */
	/* printf("CH1: %d\t CH2: %d\t", CH1, CH2); */
	/* printf("Vc1: %d\t Vc2: %d\n", (int)Vc1, (int)Vc2); */
	//for(int i = 0; i < 1000000; i++);

	status = accel_status();
	accel_read(&x, &y, &z);

	/* printf("id: %02x, status: %02x\t", id, status); */
	/* printf("x: %f, y: %f, z: %f\n", x, y, z); */

	// Desired acceleration in Y axis
	desired = -0.4f;
	actual = y;
	error = desired - actual;

	integral = 0.9 * integral + (error * dt);
	output = Kp * error + Ki * integral + bias;

	printf("actual: %f, error: %.5f, integral: %.5f, output: %.5f\n",
	       actual, error, integral, output);

	for(int i = 0; i < 1000000; ++i);
    }
}
