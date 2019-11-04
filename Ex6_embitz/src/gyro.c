#include "gyro.h"
#include "spi.h"

/*****************************/
/*** Gyro Control Functions ***/
/*****************************/

uint8_t gyro_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;

    GYRO_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    GYRO_CS_HIGH();

    return data;
}

void gyro_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;

    GYRO_CS_LOW();
    spi3_transmit_word((addr<<8) | data);
    while (SPI_GetReceptionFIFOStatus(SPI3)==initial_status);
    SPI3->DR;
    GYRO_CS_HIGH();
}

void gyro_reset()
{
    GYRO_CS_HIGH();
    gyro_write_reg(GYRO_CTRL_REG2, 0x00);
    gyro_write_reg(GYRO_CTRL_REG1, 0x0F);
}

uint8_t gyro_temp()
{
    return gyro_read_reg(GYRO_OUT_TEMP);
}

uint8_t gyro_status()
{
    return gyro_read_reg(GYRO_OUT_STAT);
}

uint8_t gyro_whoami()
{
    return gyro_read_reg(GYRO_WHOAMI);
}

void gyro_read(float *x, float *y, float *z)
{
    int16_t xi, yi, zi;
    xi = (gyro_read_reg(GYRO_OUT_X_H)<<8) | gyro_read_reg(GYRO_OUT_X_L);
    yi = (gyro_read_reg(GYRO_OUT_Y_H)<<8) | gyro_read_reg(GYRO_OUT_Y_L);
    zi = (gyro_read_reg(GYRO_OUT_Z_H)<<8) | gyro_read_reg(GYRO_OUT_Z_L);

    xi -= GYRO_XCAL;
    yi -= GYRO_YCAL;
    zi -= GYRO_ZCAL;

    *x = (float)xi * GYRO_SENSITIVITY;
    *y = (float)yi * GYRO_SENSITIVITY;
    *z = (float)zi * GYRO_SENSITIVITY;
}

void init_gyro()
{
    // Init chip select
    initPin(GPIOD, 2, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    //for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    gyro_reset();
}
