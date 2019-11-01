#ifndef _GPIO_H
#define _GPIO_H
#include "stm32f30x_conf.h"

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

int initPin(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t pupd, uint8_t otype);
int initPinAlternate(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t AF);
int initLed(void);
void setLed(uint8_t color);
int initJoystick(void);
uint8_t readJoystick(void);

#endif

