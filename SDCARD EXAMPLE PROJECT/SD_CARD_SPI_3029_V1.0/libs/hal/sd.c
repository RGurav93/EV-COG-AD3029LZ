/*! *****************************************************************************
 * @file:    sd.c
 * @brief:   provides command sequence for sdhc interface.
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
#include <stddef.h>		/* for 'NULL' */
#include <string.h>		/* for strlen */
#include "common.h"

#include <drivers/general/adi_drivers_general.h>

#include <drivers/spi/adi_spi.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/gpio/adi_gpio.h>
#include <drivers/dma/adi_dma.h>

#include <stddef.h>

#include <stdio.h>
#include <math.h>


#include "hal/sd.h"
#include "hal/sdcard_spi_hal.h"

#include "fsfat/diskio.h"

#ifdef ADI_DEBUG
#include <common.h>
#endif

void power_on_insertion();
bool sd_reset_sdcard();
static bool start_internal_init_proc();
static uint32_t get_card_specific_data();
static void print_card_identification();
void read_response_args(uint8_t *ret_val, uint16_t size);

uint8_t send_datablock(uint8_t *buff, uint8_t token);

SDCARD_DESC sdcard_description;

SDRESULT sd_init(void){ 
#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("Initializing SD Card \n");
#endif
  bool init_ok = false;
  spi_clockrate(SPI_LOW_SPEED);
  spi_chip_select(false);

  // wait for voltage stabilization
  power_on_insertion();                                                        
  
  // reset card into SPI mode
  // start internal controller initialization
  init_ok = sd_reset_sdcard()       
    && start_internal_init_proc()                                               
      ;
  
  spi_clockrate(SPI_HI_SPEED);
  get_card_specific_data();
  
  print_card_identification();
    
  return init_ok ? SD_OK : SD_NOTRDY;
}

SDRESULT sd_status(void){        //implementation is left to the user
  //empty function
  return SD_OK;
}

SDRESULT sd_read(uint8_t* buffer, uint32_t sector, uint32_t count){
  if(count <= 0 || sector > sdcard_description.sector_count - 1){
    return SD_PARERR;    
  }
  SDRESULT res;
  uint8_t cmd;
  uint8_t token;
  uint32_t address = 0;
  
  cmd = count > 1 ? CMD18 : CMD17;
  
  if(!sdcard_description.sdcard_hc_enabled) // byte addressing for sdsc and sdhc but ccs = 0
    address = sector * 512;
  else                                      // sector addressing
    address = sector;
  
 

  if(send_command(cmd, address) == 0){
    do{
      do {
        read_response_args(&token, 1);   
      } while(token != 0xfe);
      
      read_response_args(buffer, 512);
      buffer += 512;            
    }while(--count);    
    
    if(cmd == CMD18) send_command(CMD12, 0);
    res = SD_OK;
  } else res = SD_ERR;
      
  return res;
}

SDRESULT sd_write(const uint8_t* buffer, uint32_t sector, uint32_t count) {
  if(count <= 0 || sector > sdcard_description.sector_count - 1){
    return SD_PARERR;    
  }
  
  if(count == 1){
    if(send_command(CMD24, sector) == 0 && send_datablock((uint8_t *)buffer, 0xfe)){
       count = 0;  
      send_datablock((uint8_t *)buffer, 0xfd);
      
       for(unsigned int i=0;i<290;i++); ///refresh the SPI connection after writing
                                
        uint8_t response[5];
        read_response_args(response,4);
        send_command(CMD0,0);
        spi_terminate();
        for(unsigned int i=0;i<2900;i++);
        
        spi_config();
        sd_init();
        for(unsigned int i=0;i<2900;i++);
                              ////refresh finished
        
    }    
  } else {
    if(send_command(CMD25, sector) == 0){
      do{
        if(!send_datablock((uint8_t *)buffer, 0xfc)) break;
        buffer += 512;        
      }while(--count);
      if(!send_datablock((uint8_t *)buffer, 0xfd)) count = 1;
    }     
  }
  
  return count > 0 ? SD_ERR : SD_OK;  
}

SDRESULT sd_ioctl(uint8_t cmd, void* buffer){   
  uint8_t args[4];
  SDRESULT res = SD_ERR;
  
  switch(cmd){
  case CTRL_SYNC:
    read_response_args(args, 4);
    res = SD_OK;
    break;
  case GET_SECTOR_COUNT:
    *(uint32_t*) buffer = sdcard_description.sector_count;
    res = SD_OK;
    break;
  case GET_SECTOR_SIZE:
    *(uint32_t*) buffer = 512;
    res = SD_OK;
    break;
  case GET_BLOCK_SIZE:
    *(uint32_t*) buffer = sdcard_description.block_size;
    res = SD_OK;
    break;
  case CTRL_TRIM:    
    res = SD_OK;
    break;  
  default:
    res = SD_PARERR;
    break;
  }
  
  return res;
}

/** static commands **/

 void power_on_insertion() {  
#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("Power on initialization, 80 clk cycles \n");
#endif
  uint8_t dummybuff[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  spi_readwrite(dummybuff, NULL, 10, 0);
}

bool sd_reset_sdcard() {  
#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("Reset sdcard \n");
#endif
  
  // CMD0
  uint8_t resp = send_command(CMD0, 0);
    
#ifdef ADI_DEBUG_SPI
  if(resp != 0x01){
    DEBUG_MESSAGE("ERROR in reseting the device");
  }
#endif
  
  return (resp == 0x01) ? true : false;
}

static bool start_internal_init_proc(){
#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("Check operation conditions \n");
#endif   
  uint8_t resp = 255;
  uint8_t resp_args[4];  
  uint32_t cmd_args = 0;
 
  // check sd card operation condition
  resp = send_command(CMD8, 0x1aa);
  read_response_args(resp_args, 4);
    
  // check if valid command, sd v2
  if(resp == 0x01){   
    sdcard_description.sdcard_type = 1;    
    cmd_args = 1 << 30;
  }

#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("Start internal processor initialization");
#endif
  
  // start card initialization
  do {
    resp = send_command(CMD55, 0);
   
    resp = send_command(ACMD41, cmd_args);   
  } while((resp & 0x01));

  // retrieve OCR for sdhc cards  
  sdcard_description.sdcard_hc_enabled = false;
  
  if(sdcard_description.sdcard_type != 0){
    resp = send_command(CMD58, 0); 
    read_response_args(resp_args, 4);
    
    if(resp_args[0] & 0x40) sdcard_description.sdcard_hc_enabled = true;    
  }
  
#ifdef ADI_DEBUG_SPI
  /* if((resp_args[1] != 0xff) || (resp_args[2] != 0x80)){
    DEBUG_MESSAGE("ERROR, invalid voltage range \n");
  }  */
  if(resp == 0){
    DEBUG_MESSAGE("SD Card in valid operation \n");
  }  
#endif  
  
  return (resp == 0x00);// && (rxbuff[3] == 0x01) && (rxbuff[4] == 0xaa);
}

static uint32_t get_card_specific_data(){
  uint8_t csd_reg[18], token = 255;
  uint32_t c_size; 
  uint8_t c_size_mult, csd_type, bl_len_val;
    
  send_command(CMD9, 0);
  
  do {
    read_response_args(&token, 1);    
  } while(token != 0xfe);
  
  read_response_args(csd_reg, 18);
  
  csd_type = (csd_reg[0] & 0xc0) >> 6 ;
  
  bl_len_val = csd_reg[5] & 0x0f;    
  
  if(bl_len_val >= 9 && bl_len_val <= 11) 
    sdcard_description.block_size = (uint32_t) pow(2, bl_len_val); 
  else 
    sdcard_description.block_size = 512;
  
  if(csd_type == 0){    // sdsc 
    c_size = (csd_reg[6] & 0x03);
    c_size <<= 8;
    c_size |= (csd_reg[7]);
    c_size <<= 2;
    c_size |= ((csd_reg[8] >> 6) & 0x03);
    
    c_size_mult = (csd_reg[9] & 0x03);
    c_size_mult <<= 1;
    c_size_mult |= ((csd_reg[10] >> 7) & 0x01);  
       
    sdcard_description.sector_count = (uint64_t)(c_size + 1) * (uint64_t) pow(2, c_size_mult + 2);
    sdcard_description.device_size = sdcard_description.sector_count * sdcard_description.block_size;        
  } else {              // sdhc
    c_size = (csd_reg[7] & 0x3F);
    c_size <<= 8;
    c_size |= (csd_reg[8] & 0xFF);
    c_size <<= 8;
    c_size |= (csd_reg[9] & 0xFF);
        
    sdcard_description.device_size = (uint64_t)(c_size + 1) * 512000;
    sdcard_description.sector_count = c_size << 10; 
  }

  return sdcard_description.sector_count;
}

static void print_card_identification(){
  uint8_t cid_register[18], token = 255;
  
  send_command(CMD10, 0); 
  
  do {
    read_response_args(&token, 1);    
  } while(token != 0xfe);
    
  read_response_args(cid_register, 18); 
      
  sdcard_description.manufacturer_id = cid_register[0];
  memcpy(sdcard_description.oem_id, (char[]){cid_register[1], cid_register[2]}, 2 * sizeof sdcard_description.oem_id);
  memcpy(sdcard_description.product_name , (char[]){cid_register[3], cid_register[4], cid_register[5], cid_register[6], cid_register[7]}, 5 * sizeof sdcard_description.oem_id);
    
#ifdef ADI_DEBUG_SPI
  DEBUG_MESSAGE("SD Card details");
  DEBUG_MESSAGE("manufacturer id: %x ", sdcard_description.manufacturer_id);
  DEBUG_MESSAGE("oem id: %s ", sdcard_description.oem_id);
  DEBUG_MESSAGE("product code: %s ", sdcard_description.product_name);
  DEBUG_MESSAGE("sd card type : %s", sdsc ? "SDHC" : "SDSC");
  
  DEBUG_MESSAGE("memory size: %u MB ", (uint64_t) sdcard_description.device_size/1024000);
  DEBUG_MESSAGE("sector count: %u ", sdcard_description.sector_count);
  DEBUG_MESSAGE("block size: %u ", sdcard_description.block_size);
#endif
}

 uint8_t send_command(uint8_t cmd, uint32_t args){   
  uint8_t tx_frame[7] = {0x40, 0, 0, 0, 0, 1, 0xff};
  
  uint8_t timeout_counter = 0, loop_count;
  uint8_t resp = 255;
  
  bool busy = true;
  
  // cmd
  tx_frame[0] = 0x40 + cmd;
   
  // arguments
  tx_frame[1] = (args >> 24) & 0xff;
  tx_frame[2] = (args >> 16) & 0xff;
  tx_frame[3] = (args >> 8) & 0xff;
  tx_frame[4] = args & 0xff; 
  
  // crc
  if(cmd == CMD0) tx_frame[5] = 0x95;  
  if(cmd == CMD8) tx_frame[5] = 0x87;
  
  do{   
    spi_chip_select(true);
 //  for(unsigned int k=0;k<1000;k++);
    spi_readwrite(tx_frame, &resp, 7, 1);
  //  for(unsigned int k=0;k<1000;k++);
    spi_chip_select(false);
  //  for(unsigned int k=0;k<100;k++);
    
    loop_count = 0;    
    while(resp == 0xff){
      read_response_args(&resp, 1);
      if(++loop_count == 8) break;
    }    
    
    busy = ((resp & 0x80) == 0x80)? true : false;    
  } while(busy && timeout_counter++ < 5);    
  
  return resp;
}

 void read_response_args(uint8_t *ret_val, uint16_t size){
  spi_chip_select(true); 
   //  for(unsigned int k=0;k<100;k++);

  spi_readwrite(NULL, ret_val, 0, size);  
  //   for(unsigned int k=0;k<100;k++);

  spi_chip_select(false);
 // for(unsigned int k=0;k<100;k++);
}


uint8_t send_datablock(uint8_t *buff, uint8_t token){
  uint8_t resp = 0, dummy_crc[] = {0xff, 0xff};
    
  do{
    spi_chip_select(true); 
    read_response_args(&resp, 1);
    spi_chip_select(false);
  } while (resp != 0xff);
  
  spi_chip_select(true); 
  spi_readwrite(&token, NULL, 1, 0);  
  spi_chip_select(false);
  
  if(token != 0xfd){
    spi_chip_select(true); 
    spi_readwrite(buff, NULL, 512, 0);  
    spi_readwrite(dummy_crc, NULL, 2, 0);
    spi_chip_select(false);
    
    read_response_args(&resp, 1);
    if((resp & 0x1f) == 0x05) resp = 1; else resp = 0;
  } else resp = 1;
   
  return resp;
}

