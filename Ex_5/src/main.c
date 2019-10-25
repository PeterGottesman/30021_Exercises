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
    uint8_t id;
    init_usb_uart(9600);
    init_spi_gyro();
    printf("whoami: %x\n", gyro_whoami());

    x = y = z = 0;
    id = 0;
    while(1)
    {
	//id = gyro_whoami();
	//gyro_read(&x, &y, &z);
	x = gyro_read_reg(GYRO_OUT_X_L);
	printf("x: %x\n", x);
	//printf("x: %04x, y: %04x, z: %04x, whoami: %02x\n ", x, y, z, id);
	//for(int i = 0; i < 10000; ++i);
    }
}
