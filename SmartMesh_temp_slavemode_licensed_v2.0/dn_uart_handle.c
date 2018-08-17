/*! *****************************************************************************
 * @file    dn_uart_handle.c
 * @brief   Application software for handling UART intialization and UART operations.
 *  
 *
 * @note    NA.
 -----------------------------------------------------------------------------
Copyright (c) 2017 Analog Devices, Inc.

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

********************************************************************************/

/*=========================== Defines =========================================*/
#define Rx_buffer_size   1
#define Tx_buffer_size   1
#define DMA_mode         0 
#define Rx_Buffer_Full   0x05
#define Div_C            2
#define Div_M            3
#define Div_N            1078
#define Uart_OSR         3

/*=========================== Includes =========================================*/
#include <stdio.h>
#include <drivers/uart/adi_uart.h>
#include <drivers/i2c/adi_i2c.h>
#include "common.h"
#include <drivers/pwr/adi_pwr.h>
#include "dn_uart.h"
#include "adi_callback.h"
#include "SmartMesh_RF_cog_temp_example.h"
#include "drivers/gpio/adi_gpio.h" 


/*PinMux prototyping*/
extern void adi_initpinmux(void);

/* Handle for the UART device. */
static ADI_UART_HANDLE hDevice;

/* Memory for the UART driver. */
static uint8_t UartDeviceMem[ADI_UART_MEMORY_SIZE];
static uint8_t gpioMemory[ADI_GPIO_MEMORY_SIZE]; 
uint32_t pHwError_uart;


/*=========================== Packet Buffer ===================================*/
char inbuf_mote_boot[16];
/*=========================== Variables =======================================*/
typedef struct 
{
   dn_uart_rxByte_cbt   ipmt_uart_rxByte_cb;
} dn_uart_vars_t;

dn_uart_vars_t dn_uart_vars;

/*Buffer variables*/
extern int head,tail,next,max_len;
extern uint8_t  buffer_uart[buffer_size];
extern bool full,empty;

/*Call_back submit variable*/
uint8_t *Rx_buff;


/* data array statics for temp sensor */
uint8_t rxData[DATASIZE_i2c];

/* device memory */
uint8_t devMem[ADI_I2C_MEMORY_SIZE];

/* device handle */
ADI_I2C_HANDLE i2cDevice;

/* addressing/command phase "prologue" array */
uint8_t prologueData[5];

/* transaction structure */
ADI_I2C_TRANSACTION xfr;

/* result */
ADI_I2C_RESULT result = ADI_I2C_SUCCESS;

/* HW Error result */
uint32_t hwError_i2c;

/*=========================== Flush ===========================================*/  

void dn_uart_txFlush()
{
  /*noting here*/
}
/*=========================== Receive ISR  ====================================*/                  
/* SmartMesh_RF_cog_receive_ISR handles the interrupt for Rx_buffer full and places the incoming data in a buffer,byte oriented receive implemented */

void SmartMesh_RF_cog_receive_ISR(void *pcbParam,uint32_t Event,void *pArg)
{ 
  next=head+1;

  if(next>=max_len)
     next=0;
  if(next==tail)
     full=true;
  else
  { 
     buffer_uart[head]=*pREG_UART0_RX;
     //adi_uart_Read(hDevice, Rx_buff, 1, DMA_mode, &pHwError_uart);
    // buffer_uart[head]=*Rx_buff;
     head=next;
  }
}

/*=============================Buffer handle ==================================*/
void Smartmesh_RF_cog_receive(void)
{
  dn_uart_vars.ipmt_uart_rxByte_cb(buffer_uart[tail]);                          /* This function is called from the application level */
}

/*=========================== Uart Initialization  ============================*/

/* dn_uart_init  Initialises the uart and registers for the Rx buffer full call back */
void dn_uart_init(dn_uart_rxByte_cbt rxByte_cb)
{

   /*call back function*/
   dn_uart_vars.ipmt_uart_rxByte_cb = rxByte_cb;
  


   /* configure UART */
   
   /* Pinmux initialization. */
   adi_initpinmux();
        
                    
   /* Power initialization. */
   adi_pwr_Init(); 
            
              
   /* System clock initialization. */
   adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1u);
              
   /* Peripheral clock initialization. */
   adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1u);
        
   /* gpio initialization. */
   adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);
        
   /*enabling PWM mode*/
   adi_gpio_OutputEnable( ADI_GPIO_PORT2, ADI_GPIO_PIN_0, true);                /* ADP5300 PWM GPIO */
   adi_gpio_SetHigh( ADI_GPIO_PORT2, ADI_GPIO_PIN_0);                           /* enabling ADP5300 PWM mode */ 

   /*enabling RF reset*/
   adi_gpio_OutputEnable( ADI_GPIO_PORT2, ADI_GPIO_PIN_9, true); 
   adi_gpio_SetHigh( ADI_GPIO_PORT2, ADI_GPIO_PIN_9); 
   
   /*Disabling flow control*/
   adi_gpio_OutputEnable( ADI_GPIO_PORT0, ADI_GPIO_PIN_3, true); 
   adi_gpio_SetLow( ADI_GPIO_PORT0, ADI_GPIO_PIN_3);
   
   /*Time sych packet */
   adi_gpio_OutputEnable( ADI_GPIO_PORT1, ADI_GPIO_PIN_11, true); 
   adi_gpio_SetHigh( ADI_GPIO_PORT1, ADI_GPIO_PIN_11);

   /* Open the bidirectional UART device. */
   adi_uart_Open(UART_DEVICE_NUM, ADI_UART_DIR_BIDIRECTION, UartDeviceMem, ADI_UART_MEMORY_SIZE, &hDevice);
        

   /* Buad rate initialization. */
   adi_uart_ConfigBaudRate(hDevice,Div_C,Div_M,Div_N,Uart_OSR);                 /* 15200 baud rate */
   
   
/*===========================I2C initialisation================================*/  
   /* enabling gpio for i2c */
   adi_gpio_OutputEnable( ADI_GPIO_PORT1, ADI_GPIO_PIN_12, true); 
   adi_gpio_SetHigh( ADI_GPIO_PORT1, ADI_GPIO_PIN_12); 
    
   adi_gpio_OutputEnable( ADI_GPIO_PORT2, ADI_GPIO_PIN_2, true); 
   adi_gpio_SetLow( ADI_GPIO_PORT2, ADI_GPIO_PIN_2); 
    
   adi_i2c_Open(0, &devMem, ADI_I2C_MEMORY_SIZE, &i2cDevice);   /* I2c */
   adi_i2c_Reset(i2cDevice); 
   adi_i2c_SetBitRate(i2cDevice, 400000);
   adi_i2c_SetSlaveAddress(i2cDevice, 0x48);
       
   prologueData[0]     = 0x0b;                                  /* address of ID register */
   xfr.pPrologue       = &prologueData[0];
   xfr.nPrologueSize   = 1;
   xfr.pData           = rxData;
   xfr.nDataSize       = 1;
   xfr.bReadNotWrite   = true;
   xfr.bRepeatStart    = true;

   /* clear chip ID readback value in receive buffer */
   rxData[0] = 0;

   /* blocking read */
   adi_i2c_ReadWrite(i2cDevice, &xfr, &hwError_i2c);

   
/*=========================== Debug_wait=======================================*/

   adi_uart_Read(hDevice,inbuf_mote_boot,16,0,&pHwError_uart);                       /* mote boot event */ 
   
/*=============================================================================*/

   /*Buffer Variable initialisation*/
   head = 0;
   tail = 0;
   max_len = buffer_size;
   next = 0;
   empty = false;
   full = false;
        
   /*Register for call back and call back buffer submission*/
   *pREG_UART0_IEN = Rx_Buffer_Full;                                            /* enabling receive buffer full interrupt */
   Rx_buff=(uint8_t*)pREG_UART0_RX;  
   adi_uart_RegisterCallback(hDevice, SmartMesh_RF_cog_receive_ISR, NULL);      /* Registering call back */
   adi_uart_SubmitRxBuffer(hDevice, Rx_buff, Rx_buffer_size, DMA_mode);  
}        

/*=========================== Transmission ========================================*/

/*dn_uart_txByte is called by the C-library for transmission of packets,byte oriented transfer is implemented */

   void dn_uart_txByte(uint8_t byte)
{ 
       adi_uart_Write(hDevice, &byte, Tx_buffer_size, DMA_mode, &pHwError_uart);     /* Transmission of a byte done here */
}
 