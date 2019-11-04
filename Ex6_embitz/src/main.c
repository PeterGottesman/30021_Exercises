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
    float acc_x, acc_y, acc_z;
    float mag_x, mag_y, mag_z;
    float gyro_x, gyro_y, gyro_z;
    uint8_t acc_id, mag_id, gyro_id;

    init_usb_uart(9600);
    spi3_init();
    init_mag();
    init_accel();
    init_gyro();

    acc_id = accel_whoami();
    mag_id = mag_whoami();
    gyro_id = gyro_whoami();

    acc_x = acc_y = acc_z = 0;
    mag_x = mag_y = mag_z = 0;
    gyro_x= gyro_y= gyro_z= 0;

    while(1)
    {
	//status = accel_status();
	accel_read(&acc_x, &acc_y, &acc_z);
	mag_read(&mag_x, &mag_y, &mag_z);
	gyro_read(&gyro_x,&gyro_y,&gyro_z);

	printf("acc: %02x, x: %.02f, y: %.02f, z: %.02f\t", acc_id, acc_x, acc_y, acc_z);
	printf("mag: %02x, x: %.02f, y: %.02f, z: %.02f\t", mag_id, mag_x, mag_y, mag_z);
	printf("gyro: %02x, x: %.02f, y: %.02f, z: %.02f\n", gyro_id, gyro_x, gyro_y, gyro_z);

	for(int i = 0; i < 1000000; ++i);
    }
}
