#ifndef __SPI_H
#define __SPI_H
#include "stm32f30x_conf.h"


uint8_t spi3_recv_byte();
void spi3_transmit_byte(uint8_t data);
void spi3_transmit_word(uint16_t data);
void spi3_init();
#endif
