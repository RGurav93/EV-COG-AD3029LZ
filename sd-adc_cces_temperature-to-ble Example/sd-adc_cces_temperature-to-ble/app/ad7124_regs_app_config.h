/*!
 *****************************************************************************
  @file:  ad7124_reg_app_config.h

  @brief: Header for default register configuration for AD7124 application

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef AD7124_REGS_APP_CONFIG_H_
#define AD7124_REGS_APP_CONFIG_H_

#include "ad7124.h"

/*
 * Array holding the info for the AD7124 registers - address, initial value,
 * size and access type. This is the default app config on the AD7124 eval board
 */
extern struct ad7124_st_reg ad7124_regs_app_evb[AD7124_REG_NO];

#endif /* AD7124_REGS_APP_CONFIG_H_ */
