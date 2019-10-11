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

int main(void)
{
    init_usb_uart( 9600 ); // Initialize USB serial emulation at 9600 baud
    printf("Hello World!\n"); // Show the world you are alive!
    while(1){}
}
