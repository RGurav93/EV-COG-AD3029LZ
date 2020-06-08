/*!
 *****************************************************************************
  @file:  dispatcher.h

  @brief: a virtual class that takes a SampleEnginer and transmits samples

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <stddef.h>

#include "sampling_engine.h"

/*! Dispatcher Error types */
typedef enum {
	// No error from Dispatcher
	DISPATCHER_ERROR_NONE = 0u,

	// Invalid Transmit Message
	DISPATCHER_ERROR_MESSAGE = 1u,

	// Transport Unavailable
	DISPATCHER_ERROR_TRANSPORT = 2u,
} DISPATCHER_ERROR_TYPE;


class Dispatcher
{
public:
	// The dispatcher may need to idle for a time, let it do so
	virtual void Idle(const uint16_t nTimeoutMilliSeconds = 10) = 0;

	// Is the dispatcher connected to a receiver?
	virtual bool isConnected(void) = 0;

	// Dedicated methods to transmit different types of sensor data
	virtual DISPATCHER_ERROR_TYPE TransmitTemperature(const float temperatureSample,
			const uint32_t timeStamp, const uint8_t sensorID) = 0;

	virtual DISPATCHER_ERROR_TYPE TransmitAxlXYZ(const float axlX, const float axlY,
			const float axlZ,
			const uint32_t timeStamp, const uint8_t sensorID) = 0;
};



#endif /* DISPATCHER_H_ */
