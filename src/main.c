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

#define pr_dbg(fmt, ...) if (DEBUG) printf(fmt, __VA_ARGS__)

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

    uint8_t last = -1;

    char* directions[] = {"center ", "right ", "left ", "down ", "up "};
    char outstring[50];

    while(1)
    {
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
                }
            }
            last = tmp;
            printf("Joystick in direction(s) %s\n", outstring);
        }
    }
}

