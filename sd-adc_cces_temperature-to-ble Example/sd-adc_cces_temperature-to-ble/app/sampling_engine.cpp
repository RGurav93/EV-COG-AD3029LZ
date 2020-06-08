/*!
 *****************************************************************************
  @file:  sampling_engine.cpp

  @brief: samples one of an array of temperature sensors

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/
/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <base_sensor/adi_sensor_errors.h>

#include "adi_support.h"
#include "btns_leds.h"
#include "sampling_engine.h"
#include "uart_support.h"

/******************************************************************************/
/********************** Method Definitions ************************************/
/******************************************************************************/

using namespace adi_sensor_swpack;

/*!
 * @brief      SamplingEngine Constructor
 *
 * @details    create a sampling engine to acquire temperature sensor data
 *
 */
SamplingEngine::SamplingEngine()
{
	m_SensorCount = 0;
	m_activeSensor = 0;
}


/*!
 * @brief      gets temperature sample
 *
 * @details    Provides a single temperature sample from the active sensor
 *
 * @param [inout] Temperature in degrees Celsius
 *
 * @return	   SENSOR_RESULT
 *
 */
SENSOR_RESULT SamplingEngine::getTemperatureSample(float * temperature)
{
	SENSOR_RESULT eSensorResult;

	do {
		if ((eSensorResult = m_temperatureSensors[m_activeSensor]->start()) !=
		    SENSOR_ERROR_NONE) {
			DEBUG_MESSAGE("Unable to start temperature sensor.");
			break;
		}
		if ((eSensorResult =
			     m_temperatureSensors[m_activeSensor]->getTemperatureInCelsius(
				     temperature)) != SENSOR_ERROR_NONE) {
			DEBUG_MESSAGE("Unable to get temperature in celsius.");
			break;
		}
		if ((eSensorResult = m_temperatureSensors[m_activeSensor]->stop()) !=
		    SENSOR_ERROR_NONE) {
			DEBUG_MESSAGE("Unable to stop temperature sensor.");
			break;
		}
	} while(0);
	return (eSensorResult);
}


/*!
 * @brief      advances the sampling engine to the next available sensor
 *
 * @details
 *
 * @return	   none
 *
 */
void SamplingEngine::selectNextSensor()
{
	m_activeSensor += 1;
	m_activeSensor %= m_SensorCount;
}


/*!
 * @brief      adds a new sensor to sampling engine
 *
 * @details
 *
 * @param [in] Pointer to a Temperature Sensor
 *
 * @return	   SENSOR_RESULT
 *
 */
SENSOR_RESULT SamplingEngine::addSensor(Temperature* temperatureSensor)
{
	SENSOR_RESULT eSensorResult;

	// Check we're not exceeding the max allowed sensor count
	if (m_SensorCount + 1 > MAX_SENSOR_COUNT)
		return (SENSOR_ERROR_TEMP);

	m_temperatureSensors[m_SensorCount] = temperatureSensor;

	// Open the connection to the sensor
	eSensorResult = m_temperatureSensors[m_SensorCount]->open();

	if(eSensorResult != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
		Trap();
	}

	// got here, call is well
	m_SensorCount++;
	return (eSensorResult);
}
