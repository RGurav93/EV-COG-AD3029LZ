/*
 **
 ** Source file generated on September 14, 2014 at 22:01:39.	
 **
 ** Copyright (C) 2016 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the Pin Multiplexing configuration editor. Changes to the Pin Multiplexing
 ** configuration should be made by changing the appropriate options rather
 ** than editing this file.
 **
 ** Selected Peripherals
 ** --------------------
 ** SPI2 (SCLK, MISO, MOSI)
 **
 ** GPIO (unavailable)
 ** ------------------
 ** P1_02, P1_03, P1_04
 */

#include <adi_processor.h>



#define SPI0_SCLK_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<0))
#define SPI0_MOSI_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<2))
#define SPI0_MISO_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<4))
#define SPI0_RDY_PORTP1_MUX  ((uint32_t) ((uint32_t) 2<<28))
#define SPI0_CS_2_PORTP2_MUX  ((uint32_t) ((uint32_t) 2<<16))

#define I2C0_SCL0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<8))
#define I2C0_SDA0_PORTP0_MUX  ((uint16_t) ((uint16_t) 1<<10))


int32_t adi_initpinmux(void);
int32_t adi_initpinmux_i2c(void);

int32_t adi_initpinmux(void) {
    


    *((volatile uint32_t *)REG_GPIO0_CFG) = SPI0_SCLK_PORTP0_MUX | SPI0_MOSI_PORTP0_MUX
     | SPI0_MISO_PORTP0_MUX;
    *((volatile uint32_t *)REG_GPIO1_CFG) = SPI0_RDY_PORTP1_MUX;
          
    return 0;
}


int32_t adi_initpinmux_i2c(void)
{
      *((volatile uint32_t *)REG_GPIO0_CFG) = SPI0_SCLK_PORTP0_MUX | SPI0_MOSI_PORTP0_MUX
     | SPI0_MISO_PORTP0_MUX;
    *((volatile uint32_t *)REG_GPIO1_CFG) = SPI0_RDY_PORTP1_MUX;
    
   *((volatile uint32_t *)REG_GPIO0_CFG) = I2C0_SCL0_PORTP0_MUX | I2C0_SDA0_PORTP0_MUX;
      
         return 0;
  
}


