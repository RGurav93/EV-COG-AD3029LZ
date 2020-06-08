/*!
 *****************************************************************************
  @file:  sampling_engine.cpp

  @brief:

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef SAMPLING_ENGINE_CPP_
#define SAMPLING_ENGINE_CPP_

#include <temp/adi_temperature.h>
#include <temp/adt7420/adi_adt7420.h>
#include "ad7124_temperature_sensor.h"

#define MAX_SENSOR_COUNT 3

using namespace adi_sensor_swpack;

class SamplingEngine
{
public:
	SamplingEngine();
	SENSOR_RESULT getTemperatureSample(float *);
	void selectNextSensor();
	SENSOR_RESULT addSensor(Temperature * temperatureSensor);

private:
	Temperature 		   * m_temperatureSensors[MAX_SENSOR_COUNT];
	uint8_t					m_activeSensor;
	uint8_t					m_SensorCount;
};

#endif /* SAMPLING_ENGINE_CPP_ */
