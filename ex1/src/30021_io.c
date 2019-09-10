#include "30021_io.h"

/****************************/
/*** USB Serial Functions ***/
/****************************/
void uart_putc(uint8_t c) {
    USART_SendData(USART2, (uint8_t)c);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)  == RESET){}
}

uint8_t uart_getc() {
    while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(USART2);

    if (c != 0x0D) { uart_putc(c); }

    return c;
}

int _write_r(struct _reent *r, int file, char *ptr, int len) {
    int n;

    for (n = 0; n < len; n++) {
        if (ptr[n] == '\n') {
            uart_putc('\r');
        }
        uart_putc(ptr[n] & (uint16_t)0x01FF);
    }

    return len;
}

void init_usb_uart(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= 0x00020000;    // Enable Clock for GPIO Bank A
    RCC->APB1ENR |= 0x00020000;    // Enable Clock for USART2

    // Connect pins to USART2
    GPIOA->AFR[2 >> 0x03] &= ~(0x0000000F << ((2 & 0x00000007) * 4)); // Clear alternate function for PA2
    GPIOA->AFR[2 >> 0x03] |=  (0x00000007 << ((2 & 0x00000007) * 4)); // Set alternate 7 function for PA2
    GPIOA->AFR[3 >> 0x03] &= ~(0x0000000F << ((3 & 0x00000007) * 4)); // Clear alternate function for PA3
    GPIOA->AFR[3 >> 0x03] |=  (0x00000007 << ((3 & 0x00000007) * 4)); // Set alternate 7 function for PA3

    // Configure pins PA2 and PA3 for 10 MHz alternate function
    GPIOA->OSPEEDR &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear speed register
    GPIOA->OSPEEDR |=  (0x00000001 << (2 * 2) | 0x00000001 << (3 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~(0x0001     << (2)     | 0x0001     << (3));        // Clear output type register
    GPIOA->OTYPER  |=  (0x0000     << (2)     | 0x0000     << (3));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOA->MODER   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear mode register
    GPIOA->MODER   |=  (0x00000002 << (2 * 2) | 0x00000002 << (3 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // Clear push/pull register
    GPIOA->PUPDR   |=  (0x00000001 << (2 * 2) | 0x00000001 << (3 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART2
    USART2->CR1 &= ~0x00000001; // Disable USART2
    USART2->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART2->CR2 |=  0x00000000; // Set 1 stop bits
    USART2->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART2->CR1 |=  0x00000000; // Set word length to 8 bits
    USART2->CR1 |=  0x00000000; // Set parity bits to none
    USART2->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART2->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART2->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART2CLK_Frequency;

    if ((USART2->CR1 & 0x00008000) != 0) {
      // (divider * 10) computing in case Oversampling mode is 8 Samples
      divider = (2 * apbclock) / baud;
      tmpreg  = (2 * apbclock) % baud;
    } else {
      // (divider * 10) computing in case Oversampling mode is 16 Samples
      divider = apbclock / baud;
      tmpreg  = apbclock % baud;
    }

    if (tmpreg >=  baud / 2) {
        divider++;
    }

    if ((USART2->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART2->BRR = (uint16_t)divider; // Configure baud rate
    USART2->CR1 |= 0x00000001; // Enable USART2
}

/*****************************/
/*** LCD Control Functions ***/
/*****************************/
void lcd_transmit_byte(uint8_t data) {
    GPIOB->ODR &= ~(0x0001 << 6); // CS = 0 - Start Transmission
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET) { }
    SPI_SendData8(SPI2, data);
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != SET) { }
    GPIOB->ODR |=  (0x0001 << 6); // CS = 1 - End Transmission
}

void lcd_push_buffer(uint8_t* buffer)
{
    int i = 0;

    //page 0
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Set Command
    lcd_transmit_byte(0x00);      // set column low nibble 0
    lcd_transmit_byte(0x10);      // set column hi  nibble 0
    lcd_transmit_byte(0xB0);      // set page address  0

    GPIOA->ODR |=  (0x0001 << 8); // A0 = 1 - Set Data
    for(i=0; i<128; i++) {
       lcd_transmit_byte(buffer[i]);
    }

    // page 1
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Set Command
    lcd_transmit_byte(0x00);      // set column low nibble 0
    lcd_transmit_byte(0x10);      // set column hi  nibble 0
    lcd_transmit_byte(0xB1);      // set page address  1

    GPIOA->ODR |=  (0x0001 << 8); // A0 = 1 - Set Data
    for( i = 128 ; i < 256 ; i++ ) {
       lcd_transmit_byte(buffer[i]);
    }

    //page 2
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Set Command
    lcd_transmit_byte(0x00);      // set column low nibble 0
    lcd_transmit_byte(0x10);      // set column hi  nibble 0
    lcd_transmit_byte(0xB2);      // set page address  2

    GPIOA->ODR |=  (0x0001 << 8); // A0 = 1 - Set Data
    for(i=256; i<384; i++) {
       lcd_transmit_byte(buffer[i]);
    }

    //page 3
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Set Command
    lcd_transmit_byte(0x00);      // set column low nibble 0
    lcd_transmit_byte(0x10);      // set column hi  nibble 0
    lcd_transmit_byte(0xB3);      // set page address  3

    GPIOA->ODR |=  (0x0001 << 8); // A0 = 1 - Set Data
    for(i=384; i<512; i++) {
       lcd_transmit_byte(buffer[i]);
    }
}

void lcd_reset()
{
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Reset Command/Data
    GPIOB->ODR |=  (0x0001 << 6); // CS = 1 - Reset C/S

    GPIOB->ODR &= ~(0x0001 << 14); // RESET = 0 - Reset Display
    for (uint32_t i = 0 ; i < 4680 ; i++) { asm("nop"); }; // Wait
    GPIOB->ODR |=  (0x0001 << 14); // RESET = 1 - Stop Reset
    for (uint32_t i = 0 ; i < 390000 ; i++) { asm("nop"); }; // Wait

    // Configure Display
    GPIOA->ODR &= ~(0x0001 << 8); // A0 = 0 - Set Command

    lcd_transmit_byte(0xAE);  // Turn off display
    lcd_transmit_byte(0xA2);  // Set bias voltage to 1/9

    lcd_transmit_byte(0xA0);  // Set display RAM address normal
    lcd_transmit_byte(0xC8);  // Set update direction

    lcd_transmit_byte(0x22);  // Set internal resistor ratio
    lcd_transmit_byte(0x2F);  // Set operating mode
    lcd_transmit_byte(0x40);  // Set start line address

    lcd_transmit_byte(0xAF);  // Turn on display

    lcd_transmit_byte(0x81);  // Set output voltage
    lcd_transmit_byte(0x17);  // Set contrast

    lcd_transmit_byte(0xA6);  // Set normal mode
}

void init_spi_lcd() {
    // Enable Clocks
    RCC->AHBENR  |= 0x00020000 | 0x00040000;    // Enable Clock for GPIO Banks A and B
    RCC->APB1ENR |= 0x00004000;                 // Enable Clock for SPI2

    // Connect pins to SPI2
    GPIOB->AFR[13 >> 0x03] &= ~(0x0000000F << ((13 & 0x00000007) * 4)); // Clear alternate function for PB13
    GPIOB->AFR[13 >> 0x03] |=  (0x00000005 << ((13 & 0x00000007) * 4)); // Set alternate 5 function for PB13 - SCLK
    GPIOB->AFR[15 >> 0x03] &= ~(0x0000000F << ((15 & 0x00000007) * 4)); // Clear alternate function for PB15
    GPIOB->AFR[15 >> 0x03] |=  (0x00000005 << ((15 & 0x00000007) * 4)); // Set alternate 5 function for PB15 - MOSI

    // Configure pins PB13 and PB15 for 10 MHz alternate function
    GPIOB->OSPEEDR &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (13 * 2) | 0x00000001 << (15 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (13)     | 0x0001     << (15));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (13)     | 0x0000     << (15));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000002 << (13 * 2) | 0x00000002 << (15 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (13 * 2) | 0x00000003 << (15 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (13 * 2) | 0x00000000 << (15 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Initialize REEST, nCS, and A0
    // Configure pins PB6 and PB14 for 10 MHz output
    GPIOB->OSPEEDR &= ~(0x00000003 << (6 * 2) | 0x00000003 << (14 * 2));    // Clear speed register
    GPIOB->OSPEEDR |=  (0x00000001 << (6 * 2) | 0x00000001 << (14 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOB->OTYPER  &= ~(0x0001     << (6)     | 0x0001     << (14));        // Clear output type register
    GPIOB->OTYPER  |=  (0x0000     << (6)     | 0x0000     << (14));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOB->MODER   &= ~(0x00000003 << (6 * 2) | 0x00000003 << (14 * 2));    // Clear mode register
    GPIOB->MODER   |=  (0x00000001 << (6 * 2) | 0x00000001 << (14 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR   &= ~(0x00000003 << (6 * 2) | 0x00000003 << (14 * 2));    // Clear push/pull register
    GPIOB->PUPDR   |=  (0x00000000 << (6 * 2) | 0x00000000 << (14 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)
    // Configure pin PA8 for 10 MHz output
    GPIOA->OSPEEDR &= ~0x00000003 << (8 * 2);    // Clear speed register
    GPIOA->OSPEEDR |=  0x00000001 << (8 * 2);    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOA->OTYPER  &= ~0x0001     << (8);        // Clear output type register
    GPIOA->OTYPER  |=  0x0000     << (8);        // Set output type register (0x00 - Push pull, 0x01 - Open drain)


    GPIOA->MODER   &= ~0x00000003 << (8 * 2);    // Clear mode register
    GPIOA->MODER   |=  0x00000001 << (8 * 2);    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)

    GPIOA->MODER   &= ~(0x00000003 << (2 * 2) | 0x00000003 << (3 * 2));    // This is needed for UART to work. It makes no sense.
    GPIOA->MODER   |=  (0x00000002 << (2 * 2) | 0x00000002 << (3 * 2));

    GPIOA->PUPDR   &= ~0x00000003 << (8 * 2);    // Clear push/pull register
    GPIOA->PUPDR   |=  0x00000000 << (8 * 2);    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    GPIOB->ODR |=  (0x0001 << 6); // CS = 1

    // Configure SPI2
    SPI2->CR1 &= 0x3040; // Clear CR1 Register
    SPI2->CR1 |= 0x0000; // Configure direction (0x0000 - 2 Lines Full Duplex, 0x0400 - 2 Lines RX Only, 0x8000 - 1 Line RX, 0xC000 - 1 Line TX)
    SPI2->CR1 |= 0x0104; // Configure mode (0x0000 - Slave, 0x0104 - Master)
    SPI2->CR1 |= 0x0002; // Configure clock polarity (0x0000 - Low, 0x0002 - High)
    SPI2->CR1 |= 0x0001; // Configure clock phase (0x0000 - 1 Edge, 0x0001 - 2 Edge)
    SPI2->CR1 |= 0x0200; // Configure chip select (0x0000 - Hardware based, 0x0200 - Software based)
    SPI2->CR1 |= 0x0008; // Set Baud Rate Prescaler (0x0000 - 2, 0x0008 - 4, 0x0018 - 8, 0x0020 - 16, 0x0028 - 32, 0x0028 - 64, 0x0030 - 128, 0x0038 - 128)
    SPI2->CR1 |= 0x0000; // Set Bit Order (0x0000 - MSB First, 0x0080 - LSB First)
    SPI2->CR2 &= ~0x0F00; // Clear CR2 Register
    SPI2->CR2 |= 0x0700; // Set Number of Bits (0x0300 - 4, 0x0400 - 5, 0x0500 - 6, ...);
    SPI2->I2SCFGR &= ~0x0800; // Disable I2S
    SPI2->CRCPR = 7; // Set CRC polynomial order
    SPI2->CR2 &= ~0x1000;
    SPI2->CR2 |= 0x1000; // Configure RXFIFO return at (0x0000 - Half-full (16 bits), 0x1000 - Quarter-full (8 bits))
    SPI2->CR1 |= 0x0040; // Enable SPI2

    lcd_reset();
}
