/*************************************************************************//**
 *   @file   ad7124_user_config.h
 *   @brief  Header for AD7124 user configuration file
******************************************************************************
* Copyright (c) 2020 Analog Devices, Inc.
*
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*****************************************************************************/

#ifndef _AD7124_USER_CONFIG_H_
#define _AD7124_USER_CONFIG_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "ad7124.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

extern struct ad7124_init_param sAd7124_init;
extern struct ad7124_st_reg * ad7124_register_map;

#endif //_AD7124_USER_CONFIG_H_
