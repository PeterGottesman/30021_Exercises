#ifndef __MAG_H
#define __MAG_H
#include "stm32f30x_conf.h"
#include "gpio.h"

#define MAG_CS_GPIO GPIOB
#define MAG_CS_PIN 0

#define MAG_CS_LOW() (MAG_CS_GPIO->ODR &=  ~(0x0001 << MAG_CS_PIN))
#define MAG_CS_HIGH() (MAG_CS_GPIO->ODR |=  (0x0001 << MAG_CS_PIN))
#define MAG_CTRL_REG1 0x20
#define MAG_CTRL_REG2 0x21
#define MAG_CTRL_REG3 0x22
#define MAG_CTRL_REG4 0x23
#define MAG_OUT_STAT 0x27
#define MAG_OUT_X_L 0x28
#define MAG_OUT_X_H 0x29
#define MAG_OUT_Y_L 0x2A
#define MAG_OUT_Y_H 0x2B
#define MAG_OUT_Z_L 0x2C
#define MAG_OUT_Z_H 0x2D
#define MAG_WHOAMI 0x0F

void mag_read(int16_t *x, int16_t *y, int16_t *z);
void init_mag();
//uint8_t gyro_temp();
uint8_t mag_status();
uint8_t mag_whoami();
#endif
