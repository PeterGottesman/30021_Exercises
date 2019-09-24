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
#include "lcd.h"


int main(void)

{
  init_spi_lcd();
  uint8_t fbuffer[512];
  memset(fbuffer,0x00,512);
  lcd_push_buffer(fbuffer);

  lcd_write_string("1line", fbuffer, 10, 0);
  lcd_write_string("2line", fbuffer, 0, 1);
  lcd_write_string("3line", fbuffer, 10, 2);
  lcd_write_string("4line", fbuffer, 0, 3);
  lcd_push_buffer(fbuffer);
  while(1)
  {

  }
}
