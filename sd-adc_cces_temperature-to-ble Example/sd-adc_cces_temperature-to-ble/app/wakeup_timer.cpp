/*!
 *****************************************************************************
  @file:  wakeup_timer.cpp

  @brief: wake-up timer wakes the part periodically to do some work

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <time.h>
#include <stddef.h>  /* for 'NULL' */
#include <stdio.h>   /* for scanf */
#include <string.h>  /* for strncmp */
#include <drivers/pwr/adi_pwr.h>
#include <drivers/rtc/adi_rtc.h>

#include "uart_support.h"
#include "wakeup_timer.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/* device memory */
static uint8_t rtcWakeupMem[ADI_RTC_MEMORY_SIZE];

/* device handles */
ADI_RTC_HANDLE hDevRtcWakeup = NULL;

/* hibernate exit flag */
volatile uint32_t iHibernateExitFlag;

/* time duration between/period of wakeup interrupts */
static uint16_t iHiberatePeriod = 10;

/* RTC raw time */
static time_t rawtime;

/******************************************************************************/
/********************** Function Prototypes ***********************************/
/******************************************************************************/

static ADI_RTC_RESULT wakeup_SetNextTime (void);
static ADI_RTC_RESULT rtc_Calibrate(void);
static uint32_t       BuildSeconds(void);
static void 		  rtc_ReportTime(void);

/* callback */
static void rtcWakeupCallback (void *pCBParam, uint32_t Event,
			       void *EventArg);

/******************************************************************************/
/********************** Method Definitions ************************************/
/******************************************************************************/

ADI_RTC_RESULT wakeup_initialize (void)
{
	uint32_t buildTime = BuildSeconds();
	ADI_RTC_RESULT eResult;

	/* Use both static configuration and dynamic configuration for illustrative purposes */
	do {
		eResult = adi_rtc_Open(RTC_WAKEUP_DEV_NUM, rtcWakeupMem, ADI_RTC_MEMORY_SIZE,
				       &hDevRtcWakeup);
		DEBUG_RESULT("\n Failed to open the device %04d",eResult,ADI_RTC_SUCCESS);

		eResult = adi_rtc_RegisterCallback(hDevRtcWakeup, rtcWakeupCallback,
						   hDevRtcWakeup);
		DEBUG_RESULT("\n Failed to register callback",eResult,ADI_RTC_SUCCESS);

		eResult = adi_rtc_SetCount(hDevRtcWakeup, buildTime);
		DEBUG_RESULT("Failed to set the count", eResult, ADI_RTC_SUCCESS);

		eResult = adi_rtc_SetTrim(hDevRtcWakeup, ADI_RTC_TRIM_INTERVAL,
					  ADI_RTC_TRIM_VALUE, ADI_RTC_TRIM_DIRECTION);
		DEBUG_RESULT("Failed to set the trim value",eResult,ADI_RTC_SUCCESS);


		/* force a reset to the latest build timestamp */
		DEBUG_MESSAGE("Resetting clock to latest build time...");
		eResult = adi_rtc_SetCount(hDevRtcWakeup, buildTime);
		DEBUG_RESULT("Failed to set count",eResult,ADI_RTC_SUCCESS);

		DEBUG_MESSAGE("New time is:");
		rtc_ReportTime();

		eResult = adi_rtc_Enable(hDevRtcWakeup, true);
		DEBUG_RESULT("Failed to enable the device",eResult,ADI_RTC_SUCCESS);

		/* calibrate */
		eResult = rtc_Calibrate();
		DEBUG_RESULT("Error calibrating the wake-up timer RTC", eResult,
			     ADI_RTC_SUCCESS );

	} while(0);

	return(eResult);
}


ADI_RTC_RESULT wakeup_run (uint16_t hibernatePeriod)
{
	ADI_RTC_RESULT eResult;

	/* enable RTC alarm */
	eResult = adi_rtc_EnableAlarm(hDevRtcWakeup, true);
	DEBUG_RESULT("adi_RTC_EnableAlarm failed",eResult,ADI_RTC_SUCCESS);

	/* enable alarm interrupting */
	eResult = adi_rtc_EnableInterrupts(hDevRtcWakeup, ADI_RTC_ALARM_INT, true);
	DEBUG_RESULT("adi_RTC_EnableInterrupts failed",eResult,ADI_RTC_SUCCESS);

	/* program the normal alarm */
	iHiberatePeriod = hibernatePeriod;
	wakeup_SetNextTime();

	return(eResult);
}


static ADI_RTC_RESULT wakeup_SetNextTime (void)
{
	ADI_RTC_RESULT eResult;
	uint32_t count;

	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_GetCount(hDevRtcWakeup,&count))) {
		DEBUG_RESULT("\n Failed to get wake-up RTC Count %04d",eResult,ADI_RTC_SUCCESS);
		return(eResult);
	}
	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetAlarm(hDevRtcWakeup,
					 count + iHiberatePeriod))) {
		DEBUG_RESULT("\n Failed to set wake-up RTC Alarm %04d",eResult,ADI_RTC_SUCCESS);
		return(eResult);
	}
	return(eResult);
}


static ADI_RTC_RESULT rtc_Calibrate (void)
{
	ADI_RTC_RESULT eResult= ADI_RTC_FAILURE;
#ifdef ADI_RTC_CALIBRATE

	/*

	Compute the LF crystal trim values to compensate the RTC.  This can
	come from a static measure (a frequency counter), a real-time drift measure
	based external reference.

	Commercial crystals typically run between 20-100 ppm.  As an exercise, we
	demonstrate trimming a particular crystal and board configuration in which
	we measure an untrimmed error of about +58.6ppm (0.00586%).  This corresponds
	to a raw clock about 35.5 seconds/week fast (30 minutes/year).

	Available Trim Corrections:
	    X axis: trim interval (seconds)
	    Y axis: trim value (seconds)
	    Table content: trim correction (ppm)
	  Value     16384    32768    65536   131072 (Interval)
	    0        0.00     0.00     0.00     0.00
	    1       61.04    30.52    15.26     7.63
	    2      122.07    61.04    30.52    15.26
	    3      183.11    91.55    45.78    22.89
	    4      244.14   122.07    61.04    30.52
	    5      305.18   152.59    76.29    38.15
	    6      366.21   183.11    91.55    45.78
	    7      427.25   213.62   106.81    53.41

	Referencing the trim table, we see the closest matching ppm correction for
	our example is 61.04.  In case there are different combinations yielding
	the same desired correction, we prefer the shortest trim interval (and
	smallest trim value) so as to minimize instantaneous drift.

	So we choose a trim interval of 2^14 seconds with a negative trim value of 1
	second, subtracting 1 second every 4.5 hours to "slow" the fast crystal down
	to a more reasonable rate.  This particular trim leaves a residual error of
	negative 2.44ppm (0.000244%), making the trimmed clock a tad slow (less than
	1.5 seconds/week or about 1.3 minutes/year), but much better than the
	untrimmed accuracy of 30 minutes/year.

	*/

	ADI_PWR_RESULT ePowResult;
	if( ADI_PWR_SUCCESS != (ePowResult = adi_pwr_SetLFClockMux(
			ADI_CLOCK_MUX_LFCLK_LFXTAL))) { /*  select LFXTL clock */
		DEBUG_RESULT("\n Failed to set the LF Clock Mux", ePowResult, ADI_PWR_SUCCESS);
		return(ADI_RTC_FAILURE);
	}


	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_SetTrim(hDevRtcWakeup,
					 ADI_RTC_TRIM_INTERVAL_14,ADI_RTC_TRIM_1,ADI_RTC_TRIM_SUB))) {
		DEBUG_RESULT("\n Failed to set the device %04d", eResult, ADI_RTC_SUCCESS);
		return(eResult);
	}
	if(ADI_RTC_SUCCESS != (eResult = adi_rtc_EnableTrim(hDevRtcWakeup, true))) {
		DEBUG_RESULT("\n Failed to enable the trim %04d", eResult, ADI_RTC_SUCCESS);
		return(eResult);
	}

#else
	printf("Use \"ADI_RTC_CALIBRATE\" preprocessor macro for RTC calibration.");
#endif
	return(eResult);
}


/* standard ctime (time.h) constructs */
static void rtc_ReportTime(void)
{
	/* get the RTC count through the "time" CRTL function */
	time(&rawtime);
	ctime(&rawtime);
}


void rtc_print_time(void)
{
	char buffer[128];

	/* get the RTC count through the "time" CRTL function */
	time(&rawtime);

	/* print raw count */
	sprintf (buffer, "Raw time: %d", (int)rawtime);
	DEBUG_MESSAGE(buffer);

	/* convert to UTC string and print that too */
	sprintf (buffer, "UTC time: %s", ctime(&rawtime));
	DEBUG_MESSAGE(buffer);
}


static uint32_t BuildSeconds(void)
{
	/* count up seconds from the epoch (1/1/70) to the most recent build time */

	char timestamp[] = __DATE__ " " __TIME__;
	int month_days [] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint32_t days, month = 1u, date, year, hours, minutes, seconds;
	char Month[4];

	/* parse the build time stamp */
	sscanf(timestamp, "%s %d %d %d:%d:%d", Month, (int *)&date,(int *)&year,
	       (int *)&hours, (int *)&minutes, (int *)&seconds);

	/* parse ASCII month to a value */
	if     ( !strncmp(Month, "Jan", 3 )) month = 1;
	else if( !strncmp(Month, "Feb", 3 )) month = 2;
	else if( !strncmp(Month, "Mar", 3 )) month = 3;
	else if( !strncmp(Month, "Apr", 3 )) month = 4;
	else if( !strncmp(Month, "May", 3 )) month = 5;
	else if( !strncmp(Month, "Jun", 3 )) month = 6;
	else if( !strncmp(Month, "Jul", 3 )) month = 7;
	else if( !strncmp(Month, "Aug", 3 )) month = 8;
	else if( !strncmp(Month, "Sep", 3 )) month = 9;
	else if( !strncmp(Month, "Oct", 3 )) month = 10;
	else if( !strncmp(Month, "Nov", 3 )) month = 11;
	else if( !strncmp(Month, "Dec", 3 )) month = 12;

	/* count days from prior years */
	days=0;
	for (uint32_t y=1970; y<year; y++) {
		days += 365;
		if (LEAP_YEAR(y))
			days += 1;
	}

	/* add days for current year */
	for (uint32_t m=1; m<month; m++)
		days += month_days[m-1];

	/* adjust if current year is a leap year */
	if ( (LEAP_YEAR(year) && ( (month > 2) || ((month == 2) && (date == 29)) ) ) )
		days += 1;

	/* add days this month (not including current day) */
	days += date-1;

	return (days*24*60*60 + hours*60*60 + minutes*60 + seconds);
}


/* Wakeup Callback handler */
static void rtcWakeupCallback (void *pCBParam, uint32_t Event, void *EventArg)
{
	/* process RTC interrupts (cleared by driver) */
	if( 0 != (ADI_RTC_WRITE_PEND_INT &  Event )) {
		DEBUG_MESSAGE("Got RTC interrupt callback with ADI_RTC_INT_SOURCE_WRITE_PEND status");
	}

	if( 0 != (ADI_RTC_WRITE_SYNC_INT & Event)) {
		DEBUG_MESSAGE("Got RTC interrupt callback with ADI_RTC_INT_SOURCE_WRITE_SYNC status");
	}

	if( 0 != (ADI_RTC_WRITE_PENDERR_INT &  Event)) {
		DEBUG_MESSAGE("Got RTC interrupt callback with ADI_RTC_WRITE_PENDERR_INT status");
	}

	if( 0 != (ADI_RTC_ISO_DONE_INT & Event)) {
		DEBUG_MESSAGE("Got RTC interrupt callback with ADI_RTC_INT_SOURCE_ISO_DONE status");
	}

	if( 0 != (ADI_RTC_MOD60ALM_INT & Event)) {
		DEBUG_MESSAGE("Got RTC interrupt callbackwithon ADI_RTC_INT_SOURCE_MOD60_ALARM status");
	}

	if( 0 != (ADI_RTC_ALARM_INT & Event)) {
		rtc_ReportTime();

		adi_pwr_ExitLowPowerMode(&iHibernateExitFlag);

		/* Update wakeup alarm */
		wakeup_SetNextTime();
	}
}
