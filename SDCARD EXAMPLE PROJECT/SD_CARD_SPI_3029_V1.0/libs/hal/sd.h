/*! *****************************************************************************
 * @file:    sd.h
 * @brief:   header for sd.c
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
#ifndef _SD_H_
#define _SD_H_

#include <stdint.h>


// SDCard Command Types
#define CMD0	0                       // software reset
#define CMD1    1
#define CMD8	8                       // check voltage range
#define CMD9	9                       // read CSD register
#define CMD10   10                      // card id
#define CMD12	12                      // stop to read data
#define ACMD13	13                      
#define CMD16	16
#define CMD17	17
#define CMD18	18
#define ACMD23  23
#define CMD24   24
#define CMD25   25
#define	ACMD41	41
#define CMD55	55
#define CMD58	58

typedef enum {
  SD_OK=0,
  SD_ERR,
  SD_WPT,
  SD_NOTRDY,
  SD_PARERR
} SDRESULT;

typedef enum {
  SD_ACTIVE,
  SD_IDLE,
  SD_ERASE_RST,
  SD_ILLG_CMD,
  SD_CMD_CRC_ERR,
  SD_ERASE_SEQ_ERR,
  SD_ADD_ERR,
  SD_PARAM_ERR,
  SD_BUSY
} SDRESPONSE;

typedef struct {
  uint8_t command;
  uint32_t argument;
  uint8_t checksum;
} SDCOMMAND;

typedef struct {
  // cid
  uint8_t manufacturer_id;
  char oem_id[2];
  char product_name[5];  
  uint8_t product_rev;
  char serial_num[5];
  // csd
  uint8_t csd_version:2;
  uint64_t device_size;
  uint32_t sector_count;
  uint16_t block_size;  
  
  uint8_t sdcard_type;
  uint8_t sdcard_hc_enabled:1;
} SDCARD_DESC;
uint8_t send_datablock(uint8_t *buff, uint8_t token);
void read_response_args(uint8_t *ret_val, uint16_t size);
uint8_t send_command(uint8_t cmd, uint32_t args);
SDRESULT sd_init(void);
SDRESULT sd_status(void);
SDRESULT sd_read(uint8_t* buffer, uint32_t sector, uint32_t count);
SDRESULT sd_write(const uint8_t* buffer, uint32_t sector, uint32_t count);
SDRESULT sd_ioctl(uint8_t cmd, void* buffer);

void power_on_insertion();

void sd_send_command(uint8_t cmd, uint32_t args, uint8_t *resp);

#endif //_SD_H_