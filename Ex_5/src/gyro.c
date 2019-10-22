#include "gyro.h"

/*****************************/
/*** Gyro Control Functions ***/
/*****************************/
void spi3_transmit_byte(uint8_t data) {
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI3, data);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) != SET) { }
}

uint8_t spi3_recv_byte() {
    int data;
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { }
    data = SPI_ReceiveData8(SPI3);
    while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) != SET) { }

    return data;
}

uint8_t gyro_read_reg(uint8_t addr)
{
    uint8_t data;
    addr |= 0x80;
    
    GYRO_CS_LOW();
    spi3_transmit_byte(addr);
    data = spi3_recv_byte();
    GYRO_CS_HIGH();

    return data;
}

void gyro_write_reg(uint8_t addr, uint8_t data)
{
    addr &= 0x7f;
    
    GYRO_CS_LOW();
    spi3_transmit_byte(addr);
    spi3_transmit_byte(data);
    GYRO_CS_HIGH();
}

void gyro_reset()
{
    GYRO_CS_HIGH();
    for (uint32_t i = 0 ; i < 390000 ; i++) { __asm__("nop"); }; // Wait
    gyro_write_reg(GYRO_CTRL_REG2, 0x00);
    gyro_write_reg(GYRO_CTRL_REG1, 0x0F);
}

void gyro_read(uint16_t *x, uint16_t *y, uint16_t *z)
{
    x = (gyro_read_reg(GYRO_OUT_X_H)<<8) | gyro_read_reg(GYRO_OUT_X_L);
    y = (gyro_read_reg(GYRO_OUT_Y_H)<<8) | gyro_read_reg(GYRO_OUT_Y_L);
    z = (gyro_read_reg(GYRO_OUT_Z_H)<<8) | gyro_read_reg(GYRO_OUT_Z_L);
}

void init_spi_gyro() {
    // Enable Clocks
    RCC->APB1ENR |= RCC_APB1Periph_SPI3;
    initPinAlternate(GPIOC, 10, 6);
    initPinAlternate(GPIOC, 11, 6);
    initPinAlternate(GPIOC, 12, 6);
    initPin(GPIOD, 2, PIN_MODE_OUTPUT, PIN_PUPD_PU, PIN_OTYPE_PP);
    
    // Configure SPI3
    SPI3->CR1 &= 0x3040; // Clear CR1 Register
    SPI3->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI3->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI3->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI3->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI3->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI3->CR1 |= 0x0008; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI3->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI3->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI3->CR2 |= 0x0700; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI3->I2SCFGR &= ~0x0800; // Disable I2S
    SPI3->CRCPR = 7; // Set CRC polynomial order
    SPI3->CR2 &= ~0x1000;
    SPI3->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI3->CR1 |= 0x0040; // Enable SPI3

    gyro_reset();
}
