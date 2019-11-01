#include "accel.h"
#include "spi.h"

/*****************************/
/*** Accel Control Functions ***/
/*****************************/

uint8_t accel_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;
    
    ACCEL_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    ACCEL_CS_HIGH();

    return data;
}

void accel_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;
    
    ACCEL_CS_LOW();
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    ACCEL_CS_HIGH();
}

void accel_reset()
{
    ACCEL_CS_HIGH();
    accel_write_reg(ACCEL_CTRL_REG4, 0x03);
    accel_write_reg(ACCEL_CTRL_REG1, 0x67);
}

uint8_t accel_status()
{
    return accel_read_reg(ACCEL_OUT_STAT);
}

uint8_t accel_whoami()
{
    return accel_read_reg(ACCEL_WHOAMI);
}

void accel_read(int16_t *x, int16_t *y, int16_t *z)
{
    *x = (accel_read_reg(ACCEL_OUT_X_H)<<8) | accel_read_reg(ACCEL_OUT_X_L);
    *y = (accel_read_reg(ACCEL_OUT_Y_H)<<8) | accel_read_reg(ACCEL_OUT_Y_L);
    *z = (accel_read_reg(ACCEL_OUT_Z_H)<<8) | accel_read_reg(ACCEL_OUT_Z_L);
}

void init_accel()
{
    // Init chip select
    initPin(ACCEL_CS_GPIO, ACCEL_CS_PIN, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    //for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    accel_reset();
}
