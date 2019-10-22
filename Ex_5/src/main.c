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
#include <stdio.h>

int main(void)
{
    init_usb_uart(9600);
    printf("Hello, world!\n");
  while(1)
  {
  }
}
