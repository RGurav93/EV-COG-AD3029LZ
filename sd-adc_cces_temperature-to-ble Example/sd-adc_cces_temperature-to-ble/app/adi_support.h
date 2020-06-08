/*!
 *****************************************************************************
  @file:  adi_support.h

  @brief: Declares common and useful support type functions

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef ADI_SUPPORT_H_
#define ADI_SUPPORT_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void Trap();
void delay_ms(uint16_t msDelay);

#ifdef __cplusplus
}
#endif

#endif /* ADI_SUPPORT_H_ */
