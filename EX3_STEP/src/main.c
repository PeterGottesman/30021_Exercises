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
//#include <time.h>
#define stepperround 200

uint8_t POSITIONS[4]={0b0101, 0b1001, 0b1010, 0b0110 };
volatile int wait = 1;

/*
void delay(int ms)
{
     int counter = 0;
     while (counter < ms);
}*/

void set_step(uint8_t pos)
{
    int Q1 = (pos & 1<<3)>>3;
    int Q2 = (pos & 1<<2)>>2;
    int Q3 = (pos & 1<<1)>>1;
    int Q4 = pos & 1;

    GPIOA->ODR &= ~(1<<6);
    GPIOA->ODR |= Q1 << 6;

    GPIOB->ODR &= ~(1<<2);
    GPIOB->ODR |= Q2 << 2;

    GPIOC->ODR &= ~(1<<6);
    GPIOC->ODR |= Q3 << 6;

    GPIOC->ODR &= ~(1<<8);
    GPIOC->ODR |= Q4 << 8;
}

void TIM2_IRQHandler(void)
{
    wait=0;
    TIM2->SR &= ~0x1; //Clear Interrupt bit
}

int initTimer(void)
{
    // Initialize Syscfg clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Disable and set to defaults
    TIM2->CR1 = 0x0;

    TIM2->ARR = 1260000; // Computed from formula on handout
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
    init_usb_uart(9600);
    initTimer();

    // initialize stepper motor control pins
    initPin(GPIOA, 6, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    initPin(GPIOB, 2, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    initPin(GPIOC, 6, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    initPin(GPIOC, 8, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);

    while(1)
    {
        switch(uart_getc())
        {
            case '1': // rotate one round
                for(int i=0;i<stepperround;i++)
                {
                    while(wait);
                    wait = 1;
                    set_step(POSITIONS[i%4]);
                    //delay(ms);
                }
                break;
            case '2': // rotate two rounds
               for(int i=0;i<stepperround*2;i++)
                {
                    while(wait);
                    wait = 1;
                    set_step(POSITIONS[i%4]);
                    //delay(ms);
                }
                break;
        }
    }
}
