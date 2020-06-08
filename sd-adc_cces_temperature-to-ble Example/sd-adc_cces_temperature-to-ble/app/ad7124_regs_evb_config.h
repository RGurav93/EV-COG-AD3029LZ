/*!
 *****************************************************************************
  @file:  ad7124_regs_evb_config.h

  @brief: Header for default register configuration for T type Thermocouple
  	  	  input on AD7124 eval board

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef __AD7124_REGS_EVB_CONFIG_H__
#define __AD7124_REGS_EVB_CONFIG_H__

#include "ad7124.h"

/*
 * Array holding the info for the AD7124 registers - address, initial value,
 * size and access type. This is T type thermocouple sensor default config  on
 * the AD7124 eval board
 */
extern struct ad7124_st_reg ad7124_regs_t_type_evb[AD7124_REG_NO];

#endif /* __AD7124_REGS_EVB_CONFIG_H__ */
