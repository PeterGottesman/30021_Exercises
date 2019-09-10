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

// Should color hold
volatile int offset = 0;

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

void EXTI9_5_IRQHandler(void)
{
    offset++;
    
    EXTI->PR |= EXTI_PR_PR0;
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

    // Initialize Syscfg clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG->EXTICR[1] &= ~(0xF << 4);
    SYSCFG->EXTICR[1] |= 1 << 4;

    // Trigger interrupt on line 5 on rising edge only
    EXTI->RTSR |= 1 << 5;
    EXTI->FTSR &= ~(1 << 5);

    // Enable interrupt on line 5
    EXTI->IMR |= 1 << 5;

    NVIC_SetPriority(EXTI9_5_IRQn, 1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    uint8_t last = -1;

    char* directions[] = {"center ", "right ", "left ", "down ", "up "};
    char outstring[50];

    uint8_t color = 0;
    while(1)
    {
	color = 7;
        uint8_t tmp = readJoystick();
        if (tmp != last)
        {
            int idx = 0;
            for (int i = 0; i < 5; ++i)
            {
                if ((tmp >> i) % 2 == 1)
                {
                    strcpy(&outstring[idx], directions[4-i]);
                    idx += strlen(directions[4-i]);
		    color += i+offset;
                }
            }
            last = tmp;
            printf("Joystick in direction(s) %s\n", outstring);

	    printf("Setting color to %x\n", color);
	    setLed(color);
        }
    }
}

