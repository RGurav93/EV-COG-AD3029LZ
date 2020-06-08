/*!
 *****************************************************************************
  @file:  uart_dispatcher.h

  @brief: This class transmits samples over the UART

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef UART_DISPATCHER_H_
#define UART_DISPATCHER_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "dispatcher.h"
#include "sampling_engine.h"

/******************************************************************************/
/********************** Public Declarations****************** *****************/
/******************************************************************************/

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Class Declaration *************************************/
/******************************************************************************/

class UartDispatcher : public Dispatcher
{
	// this transmits data over a UART link
public:
	/* Constructor */
	UartDispatcher();

	virtual void Idle(const uint16_t nTimeoutMilliSeconds);
	virtual bool isConnected(void);
	virtual DISPATCHER_ERROR_TYPE TransmitTemperature(const float temperatureSample,
			const uint32_t timeStamp, const uint8_t sensorID);
	virtual DISPATCHER_ERROR_TYPE TransmitAxlXYZ(const float axlX, const float axlY,
			const float axlZ,
			const uint32_t timeStamp, const uint8_t sensorID);

private:
};

#endif /* UART_DISPATCHER_H_ */
