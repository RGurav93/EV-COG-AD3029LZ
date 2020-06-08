/*************************************************************************//**
 *   @file   ad7124_user_config.c
 *   @brief  User configuration file for AD7124 device
******************************************************************************
* Copyright (c) 2020 Analog Devices, Inc.
*
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

#include "app_config.h"
#include "ad7124_user_config.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Select default register user configuration (one at a time) */

//#define AD7124_EVB_DEFAULT		// AD7124 Eval board app default config
#define AD7124_EVB_T_TYPE		// AD7124 Eval board T type thermocouple config

/*
 * Reference the device register map structure to default register config
 * selected based on the active device and register config type
 */
#ifdef AD7124_EVB_DEFAULT
#include "ad7124_regs_app_config.h"
struct ad7124_st_reg * ad7124_register_map = ad7124_regs_app_evb;
#endif

#ifdef AD7124_EVB_T_TYPE
#include "ad7124_regs_evb_config.h"
// Default register configuration for T type thermocouple input on channel 0
struct ad7124_st_reg * ad7124_register_map = ad7124_regs_t_type_evb;
#endif

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Define the ad7124 device init parameter structure */
struct ad7124_init_param sAd7124_init = {
	/* SPI init parameters */
	{
		.type = GENERIC_SPI,
		.id = 42, 				// ID, value not significant
		.max_speed_hz = 2500000,
		.mode = SPI_MODE_3,		// CPOL = 1, CPHA =1
		.chip_select = 2
	},

	/* AD7124 register map */
#ifdef AD7124_EVB_DEFAULT
	&ad7124_regs_app_evb,
#elif defined AD7124_EVB_T_TYPE
	&ad7124_regs_t_type_evb,
#endif

	.spi_rdy_poll_cnt = 10000	// Retry count for polling
};
