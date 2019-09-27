#include "stm32f30x_conf.h"
#include "gpio.h"

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

