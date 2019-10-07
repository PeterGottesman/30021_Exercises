#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f30x_conf.h"

#define FLASH_START 0x08000000
#define FLASH_END 0x08010000
#define PG31_BASE 0x0800F800
#define PGnn_SIZE 2048

/*****************************/
/*** STM32 FLASH Helper Functions ***/
/*****************************/
void init_page_flash(uint32_t baseaddr);

uint16_t read_hword_flash(uint32_t baseaddr, uint16_t offset);

uint32_t read_word_flash(uint32_t baseaddr, uint16_t offset);

float read_float_flash(uint32_t baseaddr, uint16_t offset);

void write_hword_flash(uint32_t baseaddr, uint16_t offset, uint16_t data);

void write_word_flash(uint32_t baseaddr, uint16_t offset, uint32_t data);

void write_float_flash(uint32_t baseaddr, uint16_t offset, float data);

#endif /*! _FLASH_H_ */
