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
#include "30021_io.h"
#include "gyro.h"
#include <stdio.h>

int main(void)
{
    uint16_t x,y,z;
    init_usb_uart(9600);
    init_spi_gyro();

    x = y = z = 0;
    while(1)
    {
	gyro_read(&x, &y, &z);
	printf("x: %02x, y: %02x, z: %02x\n", x, y, z);
	for(int i = 0; i < 1000000; ++i);
    }
}
