/*!
 *****************************************************************************
  @file:  uart_dispatcher.cpp

  @brief: This class transmits samples over the UART

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

#include <base_sensor/adi_sensor_errors.h>

#include "app_config.h"
#include "adi_support.h"
#include "sampling_engine.h"
#include "uart_dispatcher.h"
#include "uart_support.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

using namespace adi_sensor_swpack;

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Method Definitions ************************************/
/******************************************************************************/

/*!
 * @brief	Constructor for UART Dispatcher class
 *
 * @return  None
 *
 * @details
 */
UartDispatcher::UartDispatcher()
{
	DEBUG_MESSAGE("Starting UART Dispatcher Service..." EOL);
}


/*!
 * @brief      transmit a temperature sample
 *
 * @param 	   temperatureSample temperature value in degreesC
 *
 * @param 	   timeStamp Time stamp value for sample
 *
 * @param 	   sensorID The sensor channel identifier
 *
 * @return     Error code indication success or failure
 *
 * @details    Sends the temperature values to the uart.
 */
DISPATCHER_ERROR_TYPE UartDispatcher::TransmitTemperature(
	const float temperatureSample, const uint32_t timeStamp, const uint8_t sensorID)
{
	DEBUG_MESSAGE("Sensor ID: %d Temperature: %5.2f " EOL, sensorID,
		      temperatureSample);

	return DISPATCHER_ERROR_NONE;
}


/*!
 * @brief      transmit XYZ Accelerometer samples
 *
 * @param 	   axlX Acceleration value in X axis
 *
 * @param 	   axlY Acceleration value in Y axis
 *
 * @param 	   axlZ Acceleration value in Z axis
 *
 * @param 	   timeStamp Time stamp value for sample
 *
 * @param 	   sensorID The sensor channel identifier
 *
 * @return     Error code indication success or failure
 *
 * @details    Sends the accelerometer values to the uart.
 */
DISPATCHER_ERROR_TYPE UartDispatcher::TransmitAxlXYZ(const float axlX,
		const float axlY, const float axlZ,
		const uint32_t timeStamp, const uint8_t sensorID)
{
	DEBUG_MESSAGE("Sensor ID: %d Acceleration X: %5.2f G Y: %5.2f G "
		      "Z: %5.2f G " EOL EOL, sensorID, axlX, axlY, axlZ);

	return DISPATCHER_ERROR_NONE;
}


/*!
 * @brief      Event Idle loop
 *
 * @param  	   nTimeoutMilliSeconds Time to idle waiting for events in milliseconds
 *
 * @details    Idles, nothing to do on the UART link
 */
void UartDispatcher::Idle(const uint16_t nTimeoutMilliSeconds)
{
	// Nothing to do here except wait
	delay_ms(nTimeoutMilliSeconds);
}


/*!
 * @brief      is the dispatcher connected to a receiver
 *
 * @param  	   None
 *
 * @details    returns if UART is connected
 */
bool UartDispatcher::isConnected(void)
{
	// for the UART, it is always assumed to be connected
	return (true);
}
