/*! *****************************************************************************
 * @file:    main.c
 * @brief:   sdhc_fatfs_spi_example.
 -----------------------------------------------------------------------------
Copyright (c) 2016 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/

#include <adi_processor.h>
#include <stddef.h>		/* for 'NULL' */
#include <string.h>		/* for strlen */
#include <stdint.h>

#include <drivers/spi/adi_spi.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/gpio/adi_gpio.h>
#include <drivers/dma/adi_dma.h>

#include <drivers/general/adi_drivers_general.h>

#include "hal/sd.h"
#include "hal/sdcard_spi_hal.h"
#include "fsfat/diskio.h"
#include "fsfat/ff.h"
#include "fsfat/ffconf.h"
#include <stdint.h>

#include <ADT7420/ADT7420.h>
#include <rtc_timestamp/time_stamp.h>



int main()
{
  
 adi_initpinmux();  
common_Init();

adi_pwr_Init();
adi_pwr_SetClockDivider(ADI_CLOCK_HCLK,1);
adi_pwr_SetClockDivider(ADI_CLOCK_PCLK,1);

spi_config();


char temp_string[100];
  
FATFS fs;                                      
DIR dir;                                       
FRESULT res;                                   
FIL file;                                      
int noOfCharactersWritten;

f_mount(&fs, "", 0);   

init_RTC();

char time_string[128];
unsigned int log_count = 0;

do{
  
adi_initpinmux_i2c();
ADT7420_get_temperature(temp_string);
adi_initpinmux();

log_count++;
get_time_date(time_string);

f_opendir(&dir, "/");                   
f_open(&file,"big.txt",FA_WRITE | FA_READ|FA_OPEN_APPEND);     
f_printf(&file,"%s %d : %s *C   %S\n","temperure log",log_count,temp_string,time_string); 

f_lseek (&file,f_size(&file));

f_close(&file);                                   
f_closedir(&dir); 

}while(1);

}


