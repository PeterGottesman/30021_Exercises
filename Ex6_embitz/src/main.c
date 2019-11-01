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
#include "accel.h"
#include "mag.h"
#include "spi.h"
#include <stdio.h>

int main(void)
{
    int16_t x,y,z;
    int16_t mag_x, mag_y, mag_z;
    uint8_t id, mag_id, status;
    init_usb_uart(9600);
    spi3_init();
    init_accel();
    init_mag();
    id = accel_whoami();
    mag_id = mag_whoami();

    x = y = z = 0;
    mag_x = mag_y = mag_z = 0;

    while(1)
    {
	//status = accel_status();
	accel_read(&x, &y, &z);
	mag_read(&mag_x, &mag_y, &mag_z);

	printf("id: %02x, magid: %02x,\t", id, mag_id);
	printf("x: %d, y: %d, z: %d\t", x, y, z);
	printf("magx: %d, magy: %d, magz: %d\n", mag_x, mag_y, mag_z);
	for(int i = 0; i < 100; ++i);
    }
}
