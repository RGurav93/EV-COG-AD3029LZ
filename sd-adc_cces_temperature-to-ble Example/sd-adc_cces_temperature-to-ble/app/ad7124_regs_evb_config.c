/*!
 *****************************************************************************
  @file:  ad7124_regs_evb_config.c

  @brief: Default register configuration for T type Thermocouple input
  	  	  and RTD input on AD7124 eval board

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#include "ad7124_regs_evb_config.h"

struct ad7124_st_reg ad7124_regs_t_type_evb[AD7124_REG_NO] = {
	{0x00, 0x00, 1, 2},
	{0x01, 0x708, 2, 1},		// Set DOUT_RDYB_DEL, DATA_STAUS and Int Ref Enabled
	{0x02, 0x000000, 3, 2},		// Set default value to 0x000000
	{0x03, 0x000201, 3, 1},		// IOUT0 excitation set on AIN1 pin (for RTD)
	{0x04, 0x0010, 2, 1},		// Enable VBIAS on AN2 pin for Thermocouple
	{0x05, 0x00, 1, 2},
	{0x06, 0x000000, 3, 2},
	{0x07, 0x000044, 3, 1},
	{0x08, 0x00, 1, 2},
	{0x09, 0x8043, 2, 1},		// Chn0 : AIN2, AIN3 (T Type Thermocouple), Setup0, Chn Enabled
	{0x0A, 0x9085, 2, 1},		// Chn1 : AIN4, AIN5 (RTD), Setup1, Chn Enabled
	{0x0B, 0x0001, 2, 1},
	{0x0C, 0x0001, 2, 1},
	{0x0D, 0x0001, 2, 1},
	{0x0E, 0x0001, 2, 1},
	{0x0F, 0x0001, 2, 1},
	{0x10, 0x0001, 2, 1},
	{0x11, 0x0001, 2, 1},
	{0x12, 0x0001, 2, 1},
	{0x13, 0x0001, 2, 1},
	{0x14, 0x0001, 2, 1},
	{0x15, 0x0001, 2, 1},
	{0x16, 0x0001, 2, 1},
	{0x17, 0x0001, 2, 1},
	{0x18, 0x0001, 2, 1},
	{0x19, 0x0877, 2, 1},		// Setup0: Internal Vref, Gain=128, Bipolar, AIN Buffers Enabled
	{0x1A, 0x09E3, 2, 1},		// Setup1: REFIN1 Vref, Gain=8, Bipolar, AIN & REFIN1 Buffers Enabled
	{0x1B, 0x0860, 2, 1},
	{0x1C, 0x0860, 2, 1},
	{0x1D, 0x0860, 2, 1},
	{0x1E, 0x0860, 2, 1},
	{0x1F, 0x0860, 2, 1},
	{0x20, 0x0860, 2, 1},
	{0x21, 0x060014, 3, 1},
	{0x22, 0x060014, 3, 1},
	{0x23, 0x060180, 3, 1},
	{0x24, 0x060180, 3, 1},
	{0x25, 0x060180, 3, 1},
	{0x26, 0x060180, 3, 1},
	{0x27, 0x060180, 3, 1},
	{0x28, 0x060180, 3, 1},
	{0x29, 0x800000, 3, 1},
	{0x2A, 0x800000, 3, 1},
	{0x2B, 0x800000, 3, 1},
	{0x2C, 0x800000, 3, 1},
	{0x2D, 0x800000, 3, 1},
	{0x2E, 0x800000, 3, 1},
	{0x2F, 0x800000, 3, 1},
	{0x30, 0x800000, 3, 1},
	{0x31, 0x500000, 3, 1},
	{0x32, 0x500000, 3, 1},
	{0x33, 0x500000, 3, 1},
	{0x34, 0x500000, 3, 1},
	{0x35, 0x500000, 3, 1},
	{0x36, 0x500000, 3, 1},
	{0x37, 0x500000, 3, 1},
	{0x38, 0x500000, 3, 1},
};
