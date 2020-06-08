/*!
 *****************************************************************************
   @file:    app_config.h

   @brief:   Application config file

   @details: This modules defines the application config parameters
  -----------------------------------------------------------------------------

 Copyright (c) 2018-19, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "ad7124.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

// **** Note for User: Active Device selection ****
// Select the device type from the list of below device type defines
// e.g. #define DEV_AD7124 -> This will make AD7124 as an Active Device.
// The Active Device is default set to AD7124, if device type is not defined.

#if defined DEV_AD7124
#define ACTIVE_DEVICE ID_AD7124
#else
#warning "No active device defined. AD7124 selected as default device"
#define DEV_AD7124
#define ACTIVE_DEVICE ID_AD7124
#endif

/* RTD Precision resistance value in Ohms */
#define RTD_PRECISION_RESISTANCE	22000

/* RTD excitation current value (Use appropriate hex value from the ADC datasheet mapped
 * to excitation respective current value) */
#define RTD_EXCITATION_ON_CURRENT	0x02	/* 100uA */
#define RTD_EXCITATION_OFF_CURRENT	0x07	/* 100nA */

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
