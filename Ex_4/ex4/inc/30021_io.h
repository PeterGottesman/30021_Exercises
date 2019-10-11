#ifndef _30021_IO_H_
#define _30021_IO_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_conf.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
/****************************/
/*** USB Serial Functions ***/
/****************************/
void uart_putc(uint8_t c);
uint8_t uart_getc();
void init_usb_uart(uint32_t baud);

/*****************************/
/*** LCD Control Functions ***/
/*****************************/
void init_spi_lcd();
void lcd_transmit_byte(uint8_t data);
void lcd_push_buffer(uint8_t* buffer);
void lcd_reset();

#endif /* _30021_IO_H_ */
