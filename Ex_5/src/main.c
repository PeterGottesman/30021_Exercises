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
    uint8_t id, temp, status, count;
    init_usb_uart(9600);
    init_spi_gyro();
    id = gyro_whoami();

    count = 0;
    while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty && count < 4)
    {
	SPI3->DR;
	count++;
    }
    
    x = y = z = 0;
    while(1)
    {
	temp = gyro_temp();
	status = gyro_status();
	gyro_read(&x, &y, &z);

	printf("id: %02x, temp: %02x, status: %02x\t", id, temp, status);
	printf("x: %d, y: %d, z: %d\n", x, y, z);
	//for(int i = 0; i < 100; ++i);
    }
}
