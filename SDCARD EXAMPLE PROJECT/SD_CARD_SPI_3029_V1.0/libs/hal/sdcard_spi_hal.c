/*! *****************************************************************************
 * @file:    sdcard_spi_hal.c
 * @brief:   spi access for sd.c
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
#include "hal/sdcard_spi_hal.h"

static uint8_t SPIMem[ADI_SPI_MEMORY_SIZE];
static ADI_SPI_HANDLE spih_Dev;

static uint8_t GPIOCallbackMem[ADI_GPIO_MEMORY_SIZE];

void spi_chip_select(bool assert);
void spi_clockrate(uint8_t speed);
void spi_terminate();
static bool spi_cs_init();
static bool spi_init_peripheral();

void spi_readwrite(uint8_t *txbuff, uint8_t *rxbuff, uint16_t txsize, uint16_t rxsize);




void spi_chip_select(bool assert){
  
  
  (!assert) ? adi_gpio_SetHigh(SPI_CS_PORT, SPI_CS_PIN) : adi_gpio_SetLow(SPI_CS_PORT, SPI_CS_PIN);
  if(!assert)
  {
    for(unsigned int k=0;k<1000;k++);
  }
}

void spi_clockrate(uint8_t speed){
  adi_spi_SetBitrate(spih_Dev, (speed) ? 4000000 : 400000);
}


static bool spi_cs_init(){
  
  adi_gpio_Init(GPIOCallbackMem, ADI_GPIO_MEMORY_SIZE);
    adi_gpio_PullUpEnable(SPI_CS_PORT , SPI_CS_PIN, false);
    adi_gpio_OutputEnable(SPI_CS_PORT, SPI_CS_PIN, true);
  adi_gpio_SetHigh(SPI_CS_PORT, SPI_CS_PIN);
  
  return true;
  
}

void spi_terminate()
 {
   adi_spi_Close(spih_Dev);
 }

static bool spi_init_peripheral(){
   adi_spi_Open(SPI_DEV_NUM, SPIMem, ADI_SPI_MEMORY_SIZE, &spih_Dev);
   adi_spi_SetBitrate(spih_Dev, 400000);    
   adi_spi_SetChipSelect(spih_Dev, ADI_SPI_CS_NONE);
    adi_spi_SetIrqmode(spih_Dev, 0u);
    adi_spi_SetContinuousMode (spih_Dev, false);
   return true;
   
}

uint8_t spi_config(void){ 
  bool is_success = spi_init_peripheral() && spi_cs_init(); 
  return is_success;
}



void spi_readwrite(uint8_t *txbuff, uint8_t *rxbuff, uint16_t txsize, uint16_t rxsize){
  ADI_SPI_TRANSCEIVER spi_xcv_buff;

  if(txsize > 0){
    
    spi_xcv_buff.pTransmitter = txbuff;
    spi_xcv_buff.pReceiver = NULL;
    spi_xcv_buff.TransmitterBytes = txsize;
    spi_xcv_buff.ReceiverBytes = 0;
    spi_xcv_buff.nTxIncrement = 1;
    spi_xcv_buff.nRxIncrement = 0;
        spi_xcv_buff.bDMA = true;
    spi_xcv_buff.bRD_CTL = false;
    
    adi_spi_MasterSubmitBuffer (spih_Dev, &spi_xcv_buff);
    
//    
//    bool complete = false;
//   do{
//    adi_spi_isBufferAvailable(spih_Dev, &complete );
//    }while(!complete);
    
      
   // adi_spi_MasterReadWrite(spih_Dev, &spi_xcv_buff);
        bool complete = false;
   do{
    adi_spi_isBufferAvailable(spih_Dev, &complete );
     }while(!complete);
   
  }
  
  if(rxsize > 0){
    
    spi_xcv_buff.pTransmitter = NULL;
    spi_xcv_buff.pReceiver = rxbuff;
    spi_xcv_buff.TransmitterBytes = 0;
    spi_xcv_buff.ReceiverBytes = rxsize;
    spi_xcv_buff.nTxIncrement = 0;
    spi_xcv_buff.nRxIncrement = 1;
          spi_xcv_buff.bDMA = true;
    spi_xcv_buff.bRD_CTL = false;

    adi_spi_MasterSubmitBuffer (spih_Dev, &spi_xcv_buff);
    
//    
//      bool complete = false;
//   do{
//    adi_spi_isBufferAvailable(spih_Dev, &complete );
//    }while(!complete);
    
          //  adi_spi_MasterReadWrite(spih_Dev, &spi_xcv_buff);
    
        bool complete = false;
   do{
    adi_spi_isBufferAvailable(spih_Dev, &complete );
     }while(!complete);
   
  }  
}