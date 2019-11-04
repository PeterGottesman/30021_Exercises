#include "mag.h"
#include "spi.h"

/*****************************/
/*** Mag Control Functions ***/
/*****************************/

uint8_t mag_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;

    MAG_CS_LOW();
    spi3_transmit_word(addr << 8);
    data = spi3_recv_byte();
    MAG_CS_HIGH();

    return data;
}

void mag_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t initial_status = SPI_GetReceptionFIFOStatus(SPI3);
    addr &= 0x7f;

    MAG_CS_LOW();
    spi3_transmit_word((addr<<8) | data);

    while (SPI_GetReceptionFIFOStatus(SPI3) == initial_status);
    SPI3->DR;

    MAG_CS_HIGH();
}

void mag_reset()
{
    MAG_CS_HIGH();
    mag_write_reg(MAG_CTRL_REG3, 0x84);
    //mag_write_reg(MAG_CTRL_REG1, 0x10);
}

uint8_t mag_status()
{
    return mag_read_reg(MAG_OUT_STAT);
}

uint8_t mag_whoami()
{
    return mag_read_reg(MAG_WHOAMI);
}

void mag_read(float *x, float *y, float *z)
{
    int16_t xi, yi, zi;
    xi = (mag_read_reg(MAG_OUT_X_H)<<8) | mag_read_reg(MAG_OUT_X_L);
    yi = (mag_read_reg(MAG_OUT_Y_H)<<8) | mag_read_reg(MAG_OUT_Y_L);
    zi = (mag_read_reg(MAG_OUT_Z_H)<<8) | mag_read_reg(MAG_OUT_Z_L);

    *x = (float)xi * MAG_SENSITIVITY;
    *y = (float)yi * MAG_SENSITIVITY;
    *z = (float)zi * MAG_SENSITIVITY;
}

void init_mag()
{
    // Init chip select
    initPin(MAG_CS_GPIO, MAG_CS_PIN, PIN_MODE_OUTPUT, PIN_PUPD_NONE, PIN_OTYPE_RESET);

    // Wait for initializations
    //for (uint32_t i = 0 ; i < 5000000 ; i++) { __asm__("nop"); };
    mag_reset();
}
