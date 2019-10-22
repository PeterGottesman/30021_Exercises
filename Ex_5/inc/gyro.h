#ifndef __GYRO_H
#define __GYRO_H
#include "stm32f30x_conf.h"
#include "gpio.h"

#define GYRO_CS_LOW() (GPIOD->ODR &=  ~(0x0001 << 2))
#define GYRO_CS_HIGH() (GPIOD->ODR |=  (0x0001 << 2))
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D

void gyro_read(uint16_t *x, uint16_t *y, uint16_t *z);
void init_spi_gyro();
#endif
