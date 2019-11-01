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
    int16_t gyro_x, gyro_y, gyro_z;
    uint8_t id, mag_id,gyro_id;
    init_usb_uart(9600);
    spi3_init();
    init_mag();
    init_accel();
    init_gyro();
    id = accel_whoami();
    mag_id = mag_whoami();
    gyro_id=gyro_whoami();

    x = y = z = 0;
    mag_x = mag_y = mag_z = 0;
    gyro_x= gyro_y= gyro_z= 0;
    while(1)
    {
	//status = accel_status();
	accel_read(&x, &y, &z);
	mag_read(&mag_x, &mag_y, &mag_z);
    gyro_read(&gyro_x,&gyro_y,&gyro_z);
	printf("id: %02x, magid: %02x, gyroid: %02x\t", id, mag_id,gyro_id);
	printf("x: %d, y: %d, z: %d\t", x, y, z);
	printf("magx: %d \t magy: %d \t magz: %d \t", mag_x, mag_y, mag_z);
	printf("gyrox: %d,gyroy: %d,gyroz: %d\n",gyro_x,gyro_y,gyro_z);
	for(int i = 0; i < 100; ++i);
    }
}
