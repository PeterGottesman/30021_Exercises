#ifndef _ADC_H
#define _ADC_H

#include "stm32f30x_conf.h"

void initADC(int calib);
uint16_t ADC_measure_PA(uint8_t ch);

#endif
