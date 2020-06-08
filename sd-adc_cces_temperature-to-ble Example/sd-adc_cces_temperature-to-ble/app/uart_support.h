/*!
 *****************************************************************************
  @file:  uart_support.h

  @brief: This file transmits the data over ADuCM3029 UART link

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef UART_SUPPORT_H_
#define UART_SUPPORT_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/******************************************************************************/
/********************** Public Declarations****************** *****************/
/******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
void uart_init(void);
void uart_msg_dispatch(char *pdata);
#ifdef __cplusplus
}
#endif

extern char debug_string[150];

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Define End Of Line (EOL) */
#define EOL	"\r\n"

/* Macros to convert variadic arguments into string and dispatch over UART link */

#define DEBUG_MESSAGE(...) \
  do { \
    sprintf(debug_string,__VA_ARGS__); \
    uart_msg_dispatch(debug_string); \
  } while(0)


#define DEBUG_RESULT(s,result,expected_value) \
  do { \
    if ((result) != (expected_value)) { \
      sprintf(debug_string,"Failed: %s  %d " EOL, __FILE__,__LINE__); \
      uart_msg_dispatch(debug_string); \
      sprintf(debug_string,"%s Error Code: 0x%08X" EOL EOL "Failed" EOL,(s),(result)); \
      uart_msg_dispatch(debug_string); \
      exit(0); \
    } \
  } while (0)


/****** UART Driver configurations FOR UART 0 @115200 baud rate ****** */

/* 8 Bits word data */
#define UART0_CFG_WORD_LENGTH		3

/* 1 stop bit*/
#define UART0_CFG_STOP_BIT			1

/* No parity */
#define UART0_CFG_ENABLE_PARITY		0

/* Fractional baud rate N divide value */
#define UART0_CFG_DIVN                           1563

/* Fractional baud rate M divide value */
#define UART0_CFG_DIVM                           1

/* Fractional baud rate C divide value */
#define UART0_CFG_DIVC                           4

/* Over sample by 16 */
#define UART0_CFG_OSR                            3

/* Enable Internal FIFO */
#define UART0_CFG_ENABLE_FIFO					 1

/* TRIG Level for UART device- 1 byte to trig RX interrupt */
#define UART0_CFG_TRIG_LEVEL				 	 0

/* Continue TX while RX is active */
#define UART0_CFG_HOLD_TX						 0

/* Enable RX when TX is active */
#define UART0_CFG_DISABLE_RX                     0

/* Configure the SOUT de-assertion earlier than full stop bit(s) */
#define UART0_CFG_DEASSERTION                    0

/* Set the SOUT polarity low */
#define UART0_CFG_SOUT_POLARITY					 0

/* Disable the RX status interrupt */
#define UART0_CFG_ENABLE_RX_STATUS_INTERRUPT	 0

/* Disable the Modem status interrupt */
#define UART0_CFG_ENABLE_MODEM_STATUS_INTERRUPT	 0

#endif /* UART_SUPPORT_H_ */
