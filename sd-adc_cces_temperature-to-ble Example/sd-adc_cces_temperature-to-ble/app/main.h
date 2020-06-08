/*!
 *****************************************************************************
  @file:    main.h

  @brief:   Header file for main interface module

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018-19, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* wake-up timer Interval */
#define WAKEUP_TIMER_INTERVAL  10

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

extern volatile uint32_t iHibernateExitFlag;

void handleButton2(void);
void sampleTransmitCycle(void);

#endif /* MAIN_H_ */
