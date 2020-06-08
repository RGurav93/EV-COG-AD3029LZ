/*!
 *****************************************************************************
 * @file:    adi_time.c
 * @brief:   rtc device data file
 * @version: $Revision: 34933 $
 * @date:    $Date: 2016-06-28 07:11:25 -0400 (Tue, 28 Jun 2016) $
 *-----------------------------------------------------------------------------
 *
 Copyright (c) 2010-2016, 2020 Analog Devices, Inc. All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.
 *
 *****************************************************************************/

#include <time.h>
#include <stddef.h>  /* for 'NULL' */
#include <drivers/rtc/adi_rtc.h>
#include "wakeup_timer.h"

#ifdef CLOCKS_PER_SEC
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1   /* number of RTC clock ticks in a second */
#endif

/*
 * Implementation of standard time-related C APIs which in turn leverage the RTC */

/*
    clock_t: a integer 32-bit type supporting arithmetic operations.
    clock():
        query the RTC interface for current time.
        return seconds since last time setting.
        return -1 if "reliable" time is not available or RTC is uninitialized.
*/


clock_t clock(void)
{
	ADI_RTC_HANDLE pDev = hDevRtcWakeup;  /* OK for singular RTC... */
	uint32_t t;
	if (adi_rtc_GetCount(pDev, &t) != ADI_RTC_SUCCESS) {
		t = (uint32_t)-1;
	}
	return (clock_t)t;
}



/*
      time_t: an integral 32-bit type supporting arithmetic operations.
    __time32(__time_t *t):
        return seconds since last time set.
        return -1 if "reliable" time is not available.
        sets time to address of t, if provided.
*/

time_t __time32(time_t *t)
{
	clock_t now = clock();
	if (t) {
		*t = (time_t) now;
	}
	return (time_t) now;
}


/*
      time_t: an integral 32-bit type supporting arithmetic operations.
      time(time_t *t):
        return seconds since last time set.
        return -1 if "reliable" time is not available.
        sets time to address of t, if provided.
*/
#if !defined (__ICCARM__)
time_t time(time_t *t)
{
	clock_t now = clock();
	if (t) {
		*t = (time_t) now;
	}
	return (time_t) now;
}
#endif
