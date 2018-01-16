/*! *****************************************************************************
 * @file:    ADT7420.c
 * @brief:   simple temperature output function.
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

#include <ADT7420/ADT7420.h>

#include <stdio.h>  /* for DEBUG_MESSAGE */

/* Managed drivers and/or services include */
#include <drivers/i2c/adi_i2c.h>

#include "drivers/pwr/adi_pwr.h"

#include <drivers/gpio/adi_gpio.h>

#include "common.h"
   

#include <math.h>

#define EXPECTED_ID 0xcb
#define MAX_TEMP 100
#define MIN_TEMP 50

#ifndef SUCCESS
#define SUCCESS 0
#endif
#ifndef FAILURE
#define FAILURE 1
#endif


static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE];


extern int32_t adi_initpinmux(void);

#define DATASIZE 8

uint8_t txData[DATASIZE];
uint8_t rxData[DATASIZE];


void ADT7420_get_temperature(char* str_result);

void ftoa(float n, char *res, int afterpoint);

void ADT7420_get_temperature(char* str_result)
{
     
    uint8_t devMem[ADI_I2C_MEMORY_SIZE];

    
    ADI_I2C_HANDLE hDevice;

   
    uint8_t prologueData[5];

   
    ADI_I2C_TRANSACTION xfr;

    
    ADI_I2C_RESULT result = ADI_I2C_SUCCESS;

   
    uint32_t hwErrors;
    
    
     
     
    //  adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);
    
      adi_gpio_OutputEnable(ADI_GPIO_PORT1,ADI_GPIO_PIN_12,true);
    
  
    
       adi_gpio_SetHigh(ADI_GPIO_PORT1,ADI_GPIO_PIN_12);



        adi_i2c_Open(0, &devMem, ADI_I2C_MEMORY_SIZE, &hDevice);
           


        adi_i2c_Reset(hDevice);
           


        adi_i2c_SetBitRate(hDevice, 400000);
           


        adi_i2c_SetSlaveAddress(hDevice, 0x48);
           

        prologueData[0]     = 0;  
        xfr.pPrologue       = &prologueData[0];
        xfr.nPrologueSize   = 1;
        xfr.pData           = rxData;
        xfr.nDataSize       = 2;
        xfr.bReadNotWrite   = true;
        xfr.bRepeatStart    = true;
        

 
        adi_i2c_ReadWrite(hDevice, &xfr, &hwErrors);
    
        uint16_t temp;
        float ctemp;

        temp = (rxData[0] << 8) | (rxData[1]);
        ctemp = (temp >> 3) / 16.0;
        
        ftoa(ctemp,str_result,4);
     
        
 adi_i2c_Close(hDevice);
     
}


void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 
 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    float fpart = n - (float)ipart;
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
 



