/*! *****************************************************************************
 * @file:    time_stamp.h
 * @brief:   rtcc definitions.
 -----------------------------------------------------------------------------
Copyright (c) 2016 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/

#ifndef RTC_ALARM_H
#define RTC_ALARM_H

#include <drivers/pwr/adi_pwr.h>

/* set lowest interrupt priority (highest number)
   according to number of priority bits on-chip */
#define LOWEST_PRIORITY ((1U << __NVIC_PRIO_BITS) -1)

/* bump UART interrupt priority by one (lower number = higher priority)
   so that UART output continues during low-power hibernation test */
#define UART_PRIORITY (LOWEST_PRIORITY - 1)

/* leap-year compute macro (ignores leap-seconds) */
#define LEAP_YEAR(x) (((0==x%4)&&(0!=x%100))||(0==x%400))

/* Which RTC device to use for wakeup */
#define RTC_DEVICE_NUM_FOR_WUT 1


/* Number of RTC alarms required to be registered for successfull completion of the example */
#define ADI_RTC_NUM_ALARMS      3

/* If the RTC needs to be calibrated */
#define ADI_RTC_CALIBRATE

#define RTC_ALARM_OFFSET 5

/* Trim interval */
#define ADI_RTC_TRIM_INTERVAL    ADI_RTC_TRIM_INTERVAL_14
/* Trim operation +/- */
#define ADI_RTC_TRIM_DIRECTION   ADI_RTC_TRIM_SUB
/* Trim  value */
#define ADI_RTC_TRIM_VALUE       ADI_RTC_TRIM_1

#define RTC_DEVICE_NUM    0

///* Device handle for RTC device-1*/
//extern ADI_RTC_HANDLE hDevice0;
///* Device handle for RTC device-0 which act as wake up timer*/
//extern  ADI_RTC_HANDLE hDevice1;


void init_RTC();
void get_time_date(char* time_string);
#endif /* RTC_ALARM_H */
