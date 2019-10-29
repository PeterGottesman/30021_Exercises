#include "flash.h"

/*****************************/
/*** STM32 FLASH Helper Functions ***/
/*****************************/

void init_page_flash(uint32_t baseaddr){
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);
    FLASH_ErasePage(baseaddr);
    FLASH_Lock();
}

uint16_t read_hword_flash(uint32_t baseaddr, uint16_t offset){
    return *(uint16_t *)(baseaddr + offset * 2);
}

uint32_t read_word_flash(uint32_t baseaddr, uint16_t offset){
    return *(uint32_t *)(baseaddr + offset * 4);
}

float read_float_flash(uint32_t baseaddr, uint16_t offset){
    return *(float *)(baseaddr + offset * 4);
}

void write_hword_flash(uint32_t baseaddr, uint16_t offset, uint16_t data){
    //FLASH_Unlock();
    FLASH_ProgramHalfWord(baseaddr + offset * 2, data);
    //FLASH_Lock();
}

void write_word_flash(uint32_t baseaddr, uint16_t offset, uint32_t data){
    //FLASH_Unlock();
    FLASH_ProgramWord(baseaddr + offset * 4, data);
    //FLASH_Lock();
}

void write_float_flash(uint32_t baseaddr, uint16_t offset, float data)
{
    //FLASH_Unlock();
    FLASH_ProgramWord(baseaddr + offset * 4, *((uint32_t*)(&data)));
    //FLASH_Lock();
}
