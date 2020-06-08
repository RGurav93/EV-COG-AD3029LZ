/*!
 *****************************************************************************
  @file:  adi_support.c

  @brief: Defines common and useful support type functions

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#include "adi_support.h"

/*!
 * @brief      Trap function
 *
 * @details    In case of catastrophic errors this function is called to block
 *             infinitely.
 */
void Trap()
{
	while(1);
}

/*!
 * @brief      Basic millisecond Delay function
 *
 * @details    delay function using a for loop, so not very accurate nor
 * 				is is clock independent.
 */
void delay_ms(uint16_t msDelay)
{
	for(volatile uint32_t i = 0; i < (0x500 * msDelay); i++);
}
