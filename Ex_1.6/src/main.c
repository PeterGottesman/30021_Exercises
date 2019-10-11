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
#include "30021_io.h"

#define DEBUG 1

#define PIN_MODE_INPUT 0x0
#define PIN_MODE_OUTPUT 0x1
#define PIN_MODE_ALTERNATE 0x2
#define PIN_MODE_ANALOG 0x3
#define PIN_MODE_RESET PIN_MODE_INPUT

#define PIN_OTYPE_PP 0x0
#define PIN_OTYPE_OD 0x1
#define PIN_OTYPE_RESET PIN_OTYPE_PP

#define PIN_PUPD_NONE 0x0
#define PIN_PUPD_PU 0x1
#define PIN_PUPD_PD 0x2
#define PIN_PUPD_RESERVED 0x3

#ifndef DEBUG
#define DEBUG 0
#endif

#define pr_dbg(fmt, ...) if (DEBUG) printf(fmt, ##__VA_ARGS__)

volatile int count = 0;
volatile unsigned cap1[10], cap2[10];
volatile int output = 0;
volatile int running = 0;

int initPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t pupd, uint8_t otype)
{
    int shift;
    if (pin > 15) {
	return -1;
    }

    // Turn on clock for the port
    switch ((unsigned long)GPIO)
    {
	case (unsigned long)GPIOA:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
	    break;
	case (unsigned long)GPIOB:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOB;
	    break;
	case (unsigned long)GPIOC:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOC;
	    break;
	case (unsigned long)GPIOD:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOD;
	    break;
	case (unsigned long)GPIOE:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOE;
	    break;
	case (unsigned long)GPIOF:
	    RCC->AHBENR |= RCC_AHBPeriph_GPIOF;
	    break;
	default:
	    pr_dbg("GPIO 0x%p not recognized\n", GPIO);
	    return -1;
    }

    shift = 2 * pin;
    MODIFY_REG(GPIO->MODER, 0x3 << shift, mode << shift);
    MODIFY_REG(GPIO->PUPDR, 0x3 << shift, pupd << shift);
    if (mode == PIN_MODE_OUTPUT || mode == PIN_MODE_ALTERNATE)
    {
	MODIFY_REG(GPIO->OTYPER, 0x3 << shift, otype << shift);
    }

    return 0;
}

int initPinAlternate(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t AF)
{
    initPin(GPIO, pin, PIN_MODE_ALTERNATE, PIN_PUPD_NONE, PIN_OTYPE_RESET);
    GPIO_PinAFConfig(GPIO, pin, AF);
    return 0;
}

int initLed(void)
{
    int ret;
    
    // Red
    ret = initPin(GPIOB, 4, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    if (ret)
    {
	pr_dbg("Failed to initialize PB4\n");
	return -1;
    }

    // Green
    ret = initPin(GPIOC, 7, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    if (ret)
    {
	pr_dbg("Failed to initialize PC7\n");
	return -1;
    }

    // Blue
    ret = initPin(GPIOA, 9, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_OD);
    if (ret)
    {
	pr_dbg("Failed to initialize PA9\n");
	return -1;
    }

    return 0;
}

void setLed(uint8_t color)
{
    int red = (color & 1<<2)>>2;
    int green = (color & 1<<1)>>1;
    int blue = color & 1;

    GPIOB->ODR &= ~(1<<4);
    GPIOB->ODR |= red << 4;

    GPIOC->ODR &= ~(1<<7);
    GPIOC->ODR |= green << 7;

    GPIOA->ODR &= ~(1<<9);
    GPIOA->ODR |= blue << 9;
}

int initJoystick(void)
{
    int ret;
	    
    // Right
    ret = initPin(GPIOC, 0, PIN_MODE_INPUT, PIN_PUPD_PD, PIN_OTYPE_RESET);
    if (ret)
    {
	pr_dbg("Failed to initialize PC0\n");
	return ret;
    }

    // Up
    ret = initPin(GPIOA, 4, PIN_MODE_INPUT, PIN_PUPD_PD, PIN_OTYPE_RESET);
    if (ret)
    {
	pr_dbg("Failed to initialize PA4\n");
	return ret;
    }

    // Center
    ret = initPin(GPIOB, 5, PIN_MODE_INPUT, PIN_PUPD_PD, PIN_OTYPE_RESET);
    if (ret)
    {
	pr_dbg("Failed to initialize PB5\n");
	return ret;
    }
    
    // Left
    ret = initPin(GPIOC, 1, PIN_MODE_INPUT, PIN_PUPD_PD, PIN_OTYPE_RESET);
    if (ret)
    {
	pr_dbg("Failed to initialize PC1\n");
	return ret;
    }
    
    // Down
    ret = initPin(GPIOB, 0, PIN_MODE_INPUT, PIN_PUPD_PD, PIN_OTYPE_RESET);
    if (ret)
    {
	pr_dbg("Failed to initialize PB0\n");
	return ret;
    }

    return 0;
}

uint8_t readJoystick(void)
{
    uint8_t joyAndHappiness = 0;

    uint8_t up = (GPIOA->IDR & 0x10) >> 4;
    uint8_t down = (GPIOB->IDR & 0x1);
    uint8_t left = (GPIOC->IDR & 0x2) >> 1;
    uint8_t right = (GPIOC->IDR & 0x1);
    uint8_t center = (GPIOB->IDR & 0x20) >> 5;

    joyAndHappiness |= up | down << 1 | left << 2 | right << 3 | center << 4;

    return joyAndHappiness;
}

void TIM2_IRQHandler(void)
{
    cap1[count%10] = TIM_GetCapture1(TIM2);
    cap2[count%10] = TIM_GetCapture2(TIM2);
    output = 1;
    count++;
    setLed(0b111);
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
}

int initCounter(void)
{

    // CH1 resets on rising edge
    // CH2 Reset on falling
    // CH1-CH2 cycles low
    // CH2 cycles high
    // duty cycle = high/total
    
    // Initialize Syscfg clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    initPinAlternate(GPIOA, 5, GPIO_AF_1);

    // Disable and set to defaults
    TIM2->CR1 = 0x0;

    // Use CH1 as the input
    TIM2->CR2 &= ~TIM_CR2_TI1S;

    // Disable timer counter capture
    TIM2->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC1E);

    // Set active input for TIM2_CCR1
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_CC1S, TIM_CCMR1_CC1S_0);

    // Set CC1 polarity
    TIM2->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P);
    
    // Set active input for TIM2_CCR2
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_CC2S, TIM_CCMR1_CC2S_1);

    // Set CC2 polarity
    TIM2->CCER &= ~TIM_CCER_CC2NP;
    TIM2->CCER |= TIM_CCER_CC2P;

    // No filtering
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_IC1F, 0);

    // No prescaling
    MODIFY_REG(TIM2->CCMR1, TIM_CCMR1_IC1PSC, TIM_CCMR1_IC1PSC);

    // -------- Slave mode controller --------
    
    // Disable slave mode
    TIM2->SMCR &= ~TIM_SMCR_SMS;

    // Set TIFP1 as trigger
    MODIFY_REG(TIM2->SMCR, TIM_SMCR_TS, TIM_SMCR_TS_2 | TIM_SMCR_TS_0);

    // Set slave mode reset
    MODIFY_REG(TIM2->SMCR, TIM_SMCR_SMS, TIM_SMCR_SMS_2);

    // Enable captures
    TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E;

    // Can count up to 4294.97 seconds
    TIM2->PSC = 63;
    TIM2->ARR = ~0; // Computed from formula on handout

    // 100 cycles for 10khz
 
    // Enable
    TIM2->CR1 |= 0x1;
    TIM2->DIER |= TIM_DIER_CC1IE;

    NVIC_SetPriority(TIM2_IRQn, 1);

    return 0;
}

void EXTI9_5_IRQHandler(void)
{
    if (running)
    {
	NVIC_DisableIRQ(TIM2_IRQn);
	running = 0;
	setLed(0b011);
    }
    else
    {
	setLed(0b101);
	running = 1;
	NVIC_EnableIRQ(TIM2_IRQn);
    }
    
    EXTI_ClearITPendingBit(EXTI_Line5);
}

void EXTI0_IRQHandler(void)
{
    __disable_irq();

    memset(cap1, 0, 10 * sizeof(int));
    memset(cap2, 0, 10 * sizeof(int));
    count = 0;

    EXTI_ClearITPendingBit(EXTI_Line0);
    __enable_irq();
}

int initInputs(void)
{
    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

    // Trigger interrupt on line 5 on rising edge only
    EXTI->RTSR |= 1 << 5 | 1;
    EXTI->FTSR &= ~(1 << 5 | 1);

    // Enable interrupt on line 5
    EXTI->IMR |= 1 << 5 | 1;

    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

int main(void)
{
    int ret;

    init_usb_uart(9600);
    pr_dbg("UART Initialized\n");

    ret = initJoystick();
    if (ret)
    {
	pr_dbg("Failed to initialize joystick\n");
    } else {
	pr_dbg("Joystick Initialized\n");
    }

    ret = initLed();
    if (ret)
    {
	pr_dbg("Failed to initialize LED\n");
    } else {
	pr_dbg("LED Initialized\n");
    }

    initCounter();
    initInputs();

    setLed(0b011);

    while(1)
    {
	// Handle Joystick Input
	if (output)
	{
	    float period, duty, freq;
	    float pavg, davg;
	    int limit;

	    pavg = 0.0f;
	    davg = 0.0f;
	    limit = 10 > count ? count : 10;
	    for (int i = 0; i < limit; ++i)
	    {
		pavg += cap1[i];
		davg += cap2[i];
	    }
	    pavg /= limit;
	    davg /= limit;
	    
	    period = pavg / 1000000.0;
	    duty = davg/pavg;
	    freq = 1.0/period;
		    
	    printf("frequency(hz): %f, period: %f, duty: %f\%\n", freq, period, duty*100);
	    setLed(0b101);
	    output = 0;
	}
    }  // while true
}
