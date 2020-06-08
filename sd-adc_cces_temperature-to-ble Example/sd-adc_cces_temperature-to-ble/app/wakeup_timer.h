/*!
 *****************************************************************************
  @file:  wakeup_timer.h

  @brief: include file for managing the wake-up timer

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef WAKEUP_TIMER_H_
#define WAKEUP_TIMER_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <drivers/pwr/adi_pwr.h>
#include <drivers/rtc/adi_rtc.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* leap-year compute macro (ignores leap-seconds) */
#define LEAP_YEAR(x) (((0==x%4)&&(0!=x%100))||(0==x%400))

/* RTC device numbers */
#define RTC_WAKEUP_DEV_NUM  0

/* If the RTC needs to be calibrated */
#define ADI_RTC_CALIBRATE

/* Trim parameters */
#define ADI_RTC_TRIM_INTERVAL    ADI_RTC_TRIM_INTERVAL_14
#define ADI_RTC_TRIM_DIRECTION   ADI_RTC_TRIM_SUB
#define ADI_RTC_TRIM_VALUE       ADI_RTC_TRIM_1

/******************************************************************************/
/********************** Public Declaration ************************************/
/******************************************************************************/

/* device handles */
extern ADI_RTC_HANDLE hDevRtcWakeup;

/* Function declarations */
ADI_RTC_RESULT wakeup_initialize(void);
ADI_RTC_RESULT wakeup_run (uint16_t hibernatePeriod);
void rtc_print_time(void);

#endif /* WAKEUP_TIMER_H_ */
