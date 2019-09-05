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
#include "stm32f30x_conf.h"
#include "string.h"
#include "30021_io.h"

void initJoystick(void)
{
    RCC->AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC; // Enable clock for GPIO Port A

    // Set pin PC0 (right) to input
    GPIOC->MODER &= ~(0x0000000F); // Clear mode register
    GPIOC->MODER |= (0x000000000); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOC->PUPDR &= ~(0x0000000F); // Clear push/pull register
    GPIOC->PUPDR |= (0x0000000A); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Set pin PA4 (up) to input
    GPIOA->MODER &= ~(0x00000300); // Clear mode register
    GPIOA->MODER |= (0x00000000); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOA->PUPDR &= ~(0x00000300); // Clear push/pull register
    GPIOA->PUPDR |= (0x00000200); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)

    // Set pin PB5 (center) to input
    GPIOB->MODER &= ~(0x000000C03); // Clear mode register
    GPIOB->MODER |= (0x00000000); // Set mode register (0x00 - Input, 0x01 - Output, 0x02 - Alternate Function, 0x03 - Analog in/out)
    GPIOB->PUPDR &= ~(0x00000C03); // Clear push/pull register
    GPIOB->PUPDR |= (0x00000802); // Set push/pull register (0x00 - No pull, 0x01 - Pull-up, 0x02 - Pull-down)
}

uint8_t readJoystick(void)
{
    uint8_t joyAndHappiness = 0;

    uint8_t up = (GPIOA->IDR & 0x10) >> 4;
    uint8_t down = (GPIOB->IDR & 0x1);
    uint8_t left = (GPIOC->IDR & 0x2) >> 1;
    uint8_t right = (GPIOC->IDR & 0x1);
    uint8_t center = (GPIOB->IDR & 0x20) >> 5;

    joyAndHappiness |= up | down << 1 | left << 2 | right << 3 | center << 4;

    return joyAndHappiness;
}

int main(void)
{
    init_usb_uart(9600);
    puts("UART Initialized\n");

    initJoystick();
    puts("Joystick Initialized\n");

    uint8_t last = -1;

    char* directions[] = {"center ", "right ", "left ", "down ", "up "};
    char outstring[50];

    while(1)
    {
        uint8_t tmp = readJoystick();
        if (tmp != last)
        {
            int idx = 0;
            for (int i = 0; i < 5; ++i)
            {
                if ((tmp >> i) % 2 == 1)
                {
                    strcpy(&outstring[idx], directions[4-i]);
                    idx += strlen(directions[4-i]);
                }
            }
            last = tmp;
            printf("Joystick in direction(s) %s\n", outstring);
        }
    }
}

