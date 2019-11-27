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

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

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
    TIM17->CCR1 = 1000;

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

void setDuty(int left, int right)
{
    // Set right duty cycle
    TIM16->CCR1 = right;
	    
    // Set left duty cycle
    TIM17->CCR1 = left;
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

    float error, integral, desired, actual, derivative;
    float Kp, Ki, Kd, bias;
    float dt;

    dt = 1;
    Kp = 8.0;
    Ki = 5.0;
    Kd = 0;//-1000.0;
    bias = integral = derivative = 0;
    
    int count = 0;
    int tmp = TIM17->CCR1;
    TIM17->CCR1 = TIM16->CCR1;
    TIM16->CCR1 = tmp;

    //int states[][2] = {{600, 500}};
    //int num_states = 1;
    //int state = 0;
    int l, r;
    r = 700;
    l = 600;
    setDuty(l, r);

    float mass;

    printf("left,right,x,y,z\n");
    while (1)
    {
	status = accel_status();
	accel_read(&x, &y, &z);

	desired = 0.02f;
	actual = MAX(MIN(y, 0.3), -0.3);
	error = desired - actual;

	integral = integral + (error * dt);
	integral = MAX(MIN(integral, 0.05), -0.05);

	l += Kp * error + Ki * integral;
	r = 1300 - l;
	// > 1g
	if (l >= 1000)
	{
	    mass = -3.9398 * y + 0.5077;
        } else {
          // < 1g
          mass = -0.0032 * r + 1.5747;
        }
	
        printf("Mass %.04f grams\n", mass);

	if (l < 0) l = 0;
	if (l > 1000) l = 1000;
	setDuty(l, r);

	derivative = error;
    }
}
