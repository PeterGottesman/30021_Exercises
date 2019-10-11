/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f30x_conf.h" // STM32 config
#include "30021_io.h" // Input/output library for this course
#include <string.h>

// Write character to USART1 (for SD card)
void uart1_putc(uint8_t c) {
    USART_SendData(USART1, (uint8_t)c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)  == RESET){}
}

// Read character from USART1 (for SD card)
uint8_t uart1_getc() {
    while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){};
    uint8_t c = (uint8_t)USART_ReceiveData(USART1);

    if (c != 0x0D) { /*uart_putc(c);*/ }

    return c;
}

// USART1 communication for SD card
void init_usb_uart1(uint32_t baud) {
    setbuf(stdout, NULL); // Set stdout to disable line buffering
    setbuf(stdin,  NULL); // Set stdin  to disable line buffering

    // Enable Clocks
    RCC->AHBENR  |= 0x00020000;    // Enable Clock for GPIO Bank A
    RCC->AHBENR |= RCC_AHBPeriph_GPIOC; // Enable Clock for GPIOC
    RCC->APB2ENR |= 0x00004000;    // Enable Clock for USART1

    // Connect pins to USART1
    GPIOC->AFR[4 >> 0x03] &= ~(0x0000000F << ((4 & 0x00000007) * 4)); // Clear alternate function for PC4
    GPIOC->AFR[4 >> 0x03] |=  (0x00000007 << ((4 & 0x00000007) * 4)); // Set alternate 7 function for PC4
    GPIOC->AFR[5 >> 0x03] &= ~(0x0000000F << ((5 & 0x00000007) * 4)); // Clear alternate function for PC5
    GPIOC->AFR[5 >> 0x03] |=  (0x00000007 << ((5 & 0x00000007) * 4)); // Set alternate 7 function for PC5

    // Configure pins PC4 and PC5 for 10 MHz alternate function
    GPIOC->OSPEEDR &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));    // Clear speed register
    GPIOC->OSPEEDR |=  (0x00000001 << (4 * 2) | 0x00000001 << (5 * 2));    // set speed register (0x01 - 10 MHz, 0x02 - 2 MHz, 0x03 - 50 MHz)
    GPIOC->OTYPER  &= ~(0x0001     << (4)     | 0x0001     << (5));        // Clear output type register
    GPIOC->OTYPER  |=  (0x0000     << (4)     | 0x0000     << (5));        // Set output type register (0x00 - Push pull, 0x01 - Open drain)
    GPIOC->MODER   &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));    // Clear mode register
    GPIOC->MODER   |=  (0x00000002 << (4 * 2) | 0x00000002 << (5 * 2));    // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOC->PUPDR   &= ~(0x00000003 << (4 * 2) | 0x00000003 << (5 * 2));    // Clear push/pull register
    GPIOC->PUPDR   |=  (0x00000001 << (4 * 2) | 0x00000001 << (5 * 2));    // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    //Configure USART1
    USART1->CR1 &= ~0x00000001; // Disable USART1
    USART1->CR2 &= ~0x00003000; // Clear CR2 Configuration
    USART1->CR2 |=  0x00000000; // Set 1 stop bits
    USART1->CR1 &= ~(0x00001000 | 0x00000400 | 0x00000200 | 0x00000008 | 0x00000004); // Clear CR1 Configuration
    USART1->CR1 |=  0x00000000; // Set word length to 8 bits
    USART1->CR1 |=  0x00000000; // Set parity bits to none
    USART1->CR1 |=  0x00000004 | 0x00000008; // Set mode to RX and TX
    USART1->CR3 &= ~(0x00000100 | 0x00000200); // Clear CR3 Configuration
    USART1->CR3 |=  0x00000000; // Set hardware flow control to none

    uint32_t divider = 0, apbclock = 0, tmpreg = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus); // Get USART2 Clock frequency
    apbclock = RCC_ClocksStatus.USART1CLK_Frequency;

    if ((USART1->CR1 & 0x00008000) != 0) {
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

    if ((USART1->CR1 & 0x00008000) != 0) {
        // get the LSB of divider and shift it to the right by 1 bit
        tmpreg = (divider & (uint16_t)0x000F) >> 1;
        // update the divider value
        divider = (divider & (uint16_t)0xFFF0) | tmpreg;
    }

    USART1->BRR = (uint16_t)divider; // Configure baud rate
    USART1->CR1 |= 0x00000001; // Enable USART1
}

int uart1_write(char str[]) {
    int len = strlen(str);

    for (int n = 0; n < len; n++) {
        uart1_putc(str[n]);
    }

    return len;
}

int main(void)
{
    //init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    init_usb_uart1( 9600 ); // Initialize UART communication to SD card reader, should be half this number in config.txt

    uart1_write("count\tdata\n"); // Header of file

    char line[20];
    int wait = 0, num = 0, data = 0;

    while(1)
    {
        snprintf(line,20,"%d\t%d\n", num, data); // Content of file
        uart1_write(line); // Send to SD card

        while(wait < 1000000) { wait++; } // Delay
        wait = 0;

        num++; // Update counter
        data = (data + 13) % 400; // Update "data"
    }
}
