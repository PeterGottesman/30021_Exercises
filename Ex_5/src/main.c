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
    int16_t x,y,z;
    uint8_t id, temp, status;
    init_usb_uart(9600);
    init_spi_gyro();


    x = y = z = 0;
    //gyro_whoami();
    printf("%d\n", spi3_recv_byte());
    printf("%d\n", spi3_recv_byte());
    while ((id = gyro_whoami()) != 0xD3)
    {
	x++;
    }
    printf("%d\n", x);
    while(1)
    {
	//printf("x: %x\n", x);
	id = gyro_whoami();
	temp = gyro_temp();
	status = gyro_status();
	//id = gyro_whoami();
	gyro_read(&x, &y, &z);

	printf("id: %02x, temp: %02x, status: %02x\t", id, temp, status);
	printf("x: %d, y: %d, z: %d\n", x, y, z);
	//for(int i = 0; i < 100; ++i);
    }
}
