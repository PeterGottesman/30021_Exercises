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



int main(void)

{
  init_spi_lcd();
  uint8_t fbuffer;
  memset(fbuffer,0xAA,512); // Sets each element of the buffer to 0xAA
  lcd_push_buffer(fbuffer);

  while(1)
  {

  }
}
