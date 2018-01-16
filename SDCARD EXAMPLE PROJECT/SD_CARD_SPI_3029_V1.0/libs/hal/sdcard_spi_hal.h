/*! *****************************************************************************
 * @file:    sdcard_spi_hal.h
 * @brief:   header for sdcard_spi_hal.c
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
#ifndef _SDCARD_SPI_HAL_H_
#define _SDCARD_SPI_HAL_H_

#include <stdint.h>
#include <stddef.h>


#include "drivers/spi/adi_spi.h"
#include "drivers/gpio/adi_gpio.h"

/* modify this line according to pin and spi used*/
#define SPI_DEV_NUM     0
#define SPI_CS_PORT     ADI_GPIO_PORT2
#define SPI_CS_PIN      ADI_GPIO_PIN_8


// status for spi config
#define SPI_CONFIG_SUCCESS                  0
#define SPI_CONFIG_FAIL                     1

// spi speed: 0 - 400kHz , 1 - 4MHz
#define SPI_LOW_SPEED                       0
#define SPI_HI_SPEED                        1

#ifdef ADI_DEBUG_SPI
#include <common.h>
#endif

uint8_t spi_config(void);
void spi_read_command(uint8_t tx_size);
void spi_chip_select(bool assert);
void spi_clockrate(uint8_t speed);
void spi_readwrite(uint8_t *txbuff, uint8_t *rxbuff, uint16_t txsize, uint16_t rxsize);
void spi_terminate();

#endif //_SDCARD_SPI_HAL_H_