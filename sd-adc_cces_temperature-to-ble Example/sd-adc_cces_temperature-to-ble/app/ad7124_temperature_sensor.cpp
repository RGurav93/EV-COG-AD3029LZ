/*!
 *****************************************************************************
  @file:  ad7124_temperature_sensor.cpp

  @brief: Temperature sensing interface for AD7124 device

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

#include <stddef.h>
#include <stdio.h>
#include <math.h>

#include <base_sensor/adi_sensor_errors.h>

#include "main.h"
#include "app_config.h"

#include "thermocouple.h"
#include "ptxxx.h"

#include "adi_support.h"
#include "platform_drivers.h"
#include "common.h"

#include "ad7124_temperature_sensor.h"
#include "ad7124_user_config.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "ad7124.h"

#ifdef __cplusplus
}
#endif


/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

// These are error codes
#define CHANNEL_MISMATCH -100

/******************************************************************************/
/********************** Method Definitions ************************************/
/******************************************************************************/

namespace adi_sensor_swpack
{
/* Define the methods for AD7124 temperature sensor interface */

/**
 * @brief	Constructor for 'ad7124_temperature' class
 *
 * @param	[in] Pointer to AD7124 device instance
 *
 * @param	[in] RTD Rref (precision resistance) value (optional)
 *
 * @param	[in] RTD excitation on current value (optional)
 *
 * @param	[in] RTD excitation off current value (optional)
 *
 * @details	This is a constructor for 'ad7124_temperature' class,
 * 			which sets the AD7124 device to use as temperature sensor
 */
ad7124_temperature::ad7124_temperature(ad7124_dev * pAd7124_dev,
				       float precisionResistor,
				       ioutExcitation rtd_excitation_on_current,
				       ioutExcitation rtd_excitation_off_current)
{
	m_pAd7124 = pAd7124_dev;

	/* Select the single conversion mode for AD7124 temperature measurement */
	conversion_mode = single_conversion_mode;

	/* Set the Rref value for AD7124 RTD interface */
	this->precisionResistor = precisionResistor;

	/* Set the RTD excitation on/off current for AD7124 RTD interface */
	this->rtd_excitation_on_current = rtd_excitation_on_current;
	this->rtd_excitation_off_current = rtd_excitation_off_current;

	/* Calibrate AD7124 channels used for temperature sensing */
	if (calibrate_temp_sensor_channels() != SENSOR_ERROR_NONE) {
		DEBUG_MESSAGE("Error in AD7124 temperature sensor channels calibration\r\n");
	}
}


/**
 * @brief	Open AD7124 temperature sensor
 *
 * @return	Sensor open error status
 *
 * @details	Open AD7124 temperature sensor (no implementation, but added since
 * 			it's a pure virtual function of Temperature class)
 */
SENSOR_RESULT ad7124_temperature::open()
{
	return(SENSOR_ERROR_NONE);
}


/**
 * @brief	Start the AD7124 temperature measurement
 *
 * @return	Sensor measurement start error status
 *
 * @details	Start ADC conversion by setting the single conversion mode
 */
SENSOR_RESULT ad7124_temperature::start()
{
	int32_t		errorCode = 0;
	SENSOR_RESULT eSensorResult = SENSOR_ERROR_NONE;

	do {
		// Set continuous conversion mode
		// *Note: For single conversion mode functionality, the conversion mode
		//        is set during channel scan/acquisition
		if (conversion_mode == continuous_conversion_mode) {
			ad7124_register_map[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(
						MODE_SELECT_MASK));
			ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(
						continuous_conversion_mode);

			if ((errorCode = ad7124_write_register(m_pAd7124,
							       ad7124_register_map[AD7124_ADC_Control])) != SUCCESS) {
				DEBUG_MESSAGE("Error (%ld) setting AD7124 conversion mode.\r\n", errorCode);
				eSensorResult = SENSOR_ERROR_ADC;
				break;
			}
		}

		// Set excitation to required level for RTD sensor measurement
		if ((eSensorResult = setIout0(rtd_excitation_on_current)) !=
		    SENSOR_ERROR_NONE) {
			DEBUG_MESSAGE("Error (%ld) setting AD7124 Iout excitation.\r\n", eSensorResult);
			break;
		}

		// delay is roughly 200ms, should be enough time to let the source settle
		delay_ms(200);

	} while(0);

	return (eSensorResult);
}


/**
 * @brief	Stop the AD7124 temperature measurement
 *
 * @return	Sensor measurement stop error status
 *
 * @details	Stop ADC conversion by setting the standby mode
 */
SENSOR_RESULT ad7124_temperature::stop()
{
	int32_t		errorCode = 0;
	SENSOR_RESULT eSensorResult = SENSOR_ERROR_NONE;

	do {
		// Excitation reduced to 100nA, not 0 here so that capacitors don't discharge
		if ((eSensorResult = setIout0(rtd_excitation_off_current)) !=
		    SENSOR_ERROR_NONE) {
			DEBUG_MESSAGE("Error (%ld) setting AD7124 Iout excitation.\r\n", eSensorResult);
			break;
		}

		// Set adc standby mode
		ad7124_register_map[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(
					MODE_SELECT_MASK));
		ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(
					standby_mode);

		if ((errorCode = ad7124_write_register(m_pAd7124,
						       ad7124_register_map[AD7124_ADC_Control])) != SUCCESS) {
			DEBUG_MESSAGE("Error (%ld) setting AD7124 sleep mode.\r\n", errorCode);
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}
	} while(0);

	return (eSensorResult);
}


/**
 * @brief	Close AD7124 temperature sensor
 *
 * @return	Sensor close error status
 *
 * @details	Close AD7124 temperature sensor (no implementation, but added since
 * 			it's a pure virtual function of Temperature class)
 */
SENSOR_RESULT ad7124_temperature::close()
{
	return(SENSOR_ERROR_NONE);
}


/**
 * @brief	Return temperature.
 *
 * @param	[in, out] pTemperature : Buffer pointer where temperature is returned.
 *
 * @param	[in] sizeInBytes : Size of the buffer in bytes
 *
 * @return	Sensor temperature get error status
 *
 * @details	This method returns the current temperature in to the supplied buffer
 */
SENSOR_RESULT ad7124_temperature::getTemperature(uint8_t *pTemperature,
		const uint32_t sizeInBytes)
{
	return(SENSOR_ERROR_NONE);
}


/**
 * @brief	Returns the temperature in celsius
 *
 * @param	[in, out] pTemperature : Pointer to memory where temperature data is stored
 *
 * @return	SENSOR_RESULT
 *
 * @details	Returns the current temperature value in celsius
 */
SENSOR_RESULT ad7124_temperature::getTemperatureInCelsius(float *pTemperature)
{
	ASSERT(pTemperature != NULL);

	SENSOR_RESULT result = getNextSample(pTemperature);

	return (result);
}


/**
 * @brief	returns the temperature in fahrenheit
 *
 * @param	[in] pTemperature : Pointer to memory where temperature data is stored
 *
 * @return	SENSOR_RESULT
 *
 * @details	Returns the current temperature value in fahrenheit
 */
SENSOR_RESULT ad7124_temperature::getTemperatureInFahrenheit(
	float *pTemperature)
{
	float temperature;

	ASSERT(pTemperature != NULL);

	SENSOR_RESULT result = getNextSample(&temperature);

	*pTemperature = ((9.0F / 5.0F) * temperature) + 32.0F;

	return (result);
}


/**
 * @brief	returns temperature of a thermocouple
 *
 * @param	[in] temperatureValue : Pointer to memory where temperature val is stored
 *
 * @return	sampleError value
 *
 * @details	Returns the temperature in celsius from thermocouple.
 * 			Compensation is done using a CJC RTD sensor
 */
int32_t ad7124_temperature::getNextSample(float * temperatureValue)
{
	// Sensors are created statically as they are always required
	static PT1000	rtd_sensor;
	static Thermocouple_Type_T tcSensor;
	// Stores the last temperature readings from each channel
	static float temperatureSamples[num_of_ad7124_temp_sensors] = {-1000, -1000};
	// Stores the last ADC samples from each channel
	static uint32_t	channelSamples[num_of_ad7124_temp_sensors] = {(uint32_t)ad7124_max_ad_cnt, (uint32_t)ad7124_max_ad_cnt};

	// Error code returned when sampling a channel
	int32_t sampleError = SUCCESS;
	// last acquired sample value for a channel
	uint32_t sampleValue;

	// ADC bits converted to Ohms/mV
	float rtd_Ohms = -9999;
	float tc_mV = -9999;

	do {
		// Sample thermocouple sensor
		if (conversion_mode == single_conversion_mode) {
			sampleError = sampleChannelSC(t_thermocouple, &sampleValue);
		} else {
			sampleError = sampleChannelCC(t_thermocouple, &sampleValue);
		}

		if (sampleError != SUCCESS) {
			DEBUG_MESSAGE("Error Sampling Ch0 Thermocouple Sensor");
			break;
		} else {
			channelSamples[t_thermocouple] = sampleValue;
			tc_mV = convertRawAdcToMilliVolts(channelSamples[t_thermocouple]);
			temperatureSamples[t_thermocouple] = tcSensor.convert(tc_mV);
		}

		// Sample RTD sensor
		if (conversion_mode == single_conversion_mode) {
			sampleError = sampleChannelSC(cjc_rtd, &sampleValue);
		} else {
			sampleError = sampleChannelCC(cjc_rtd, &sampleValue);
		}

		if (sampleError != SUCCESS) {
			DEBUG_MESSAGE("Error Sampling Ch1 RTD Sensor");
			break;
		} else {
			channelSamples[cjc_rtd] = sampleValue;
			rtd_Ohms = convertRawAdcToResistance(channelSamples[cjc_rtd]);
			temperatureSamples[cjc_rtd] = rtd_sensor.convertResistanceToTemperature(
							      rtd_Ohms);
		}

		// Store the CJC converted to mV for the thermocouple
		float rtd_in_tc_mV = tcSensor.convert_inv(temperatureSamples[cjc_rtd]);

		DEBUG_MESSAGE("T-type TC (Ch0) == %ld   [[ %.2f mVolts ]]   @ %.2f degc\r\nRTD (Ch1) == %ld   [[ %.2f Ohms  ]]  @ %.2f degC  [[ %.2f mV ]]",
			      channelSamples[t_thermocouple], tc_mV, temperatureSamples[t_thermocouple],
			      channelSamples[cjc_rtd], rtd_Ohms, temperatureSamples[cjc_rtd], rtd_in_tc_mV);

		/*
		 * NOTE The simplest approach of adding the CJC temperature to TC temperature is taken here.
		 * A better method is to convert RTD back to thermocouple mV, and add that to TC value
		 * then do the thermocouple to degC conversion.
		 */

		*temperatureValue = temperatureSamples[t_thermocouple] +
				    temperatureSamples[cjc_rtd];

	} while(0);

	return (sampleError);
}


/**
 * @brief	acquires single channel sample from ADC
 *
 * @param	[in] channel to acquire
 *
 * @param	[in, out] sample value
 *
 * @return	error code 0 indicating success or else failure
 *
 * @details	This is used when ADC is in single conversion mode.  The channel to be samples is
 * 			enabled, and register polling is used to determine when the sample is available.
 */
int32_t ad7124_temperature::sampleChannelSC(uint8_t channel,
		uint32_t * pSampleValue)
{
	int32_t		errorCode;
	bool		acquired = false;
	int32_t		sample_data = 0;

	// Set the Enable bit in the CHANNEL_X configuration register
	ad7124_register_map[AD7124_Channel_0 + channel].value |=
		AD7124_CH_MAP_REG_CH_ENABLE;
	if ((errorCode = ad7124_write_register(m_pAd7124,
					       ad7124_register_map[AD7124_Channel_0 + channel]) ) != SUCCESS) {
		DEBUG_MESSAGE("Error setting Channel %d Enable bit (%ld).\r\n", channel,
			      errorCode);
	} else {
		do {
			// Trigger the single conversion mode to start sampling channels
			// *Note: Since channels are disabled and enabled at every single call of this function,
			//        the conversion mode needs to be triggered manually to start sampling channels again
			ad7124_register_map[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(
						MODE_SELECT_MASK));
			ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(
						single_conversion_mode);

			if ((errorCode = ad7124_write_register(m_pAd7124,
							       ad7124_register_map[AD7124_ADC_Control])) != SUCCESS) {
				DEBUG_MESSAGE("Error (%ld) setting AD7124 Single conversion mode\r\n",
					      errorCode);
				break;
			}

			/*
			 *  this polls the status register READY bit to determine when conversion is done
			 *  this also ensures the STATUS register value is up to date and contains the
			 *  channel that was sampled as well. No need to read STATUS separately
			 */
			if ((errorCode = ad7124_wait_for_conv_ready(m_pAd7124,
					 adc_conv_max_wait)) != SUCCESS) {
				DEBUG_MESSAGE("Error/Timeout waiting for conversion ready %ld\r\n", errorCode);
				break;
			}

			if ((errorCode = ad7124_read_data(m_pAd7124, &sample_data)) != SUCCESS) {
				DEBUG_MESSAGE("Error reading ADC Data (%ld)\r\n", errorCode);
				break;
			}

			// what channel has been read?
			uint8_t channelRead = (ad7124_register_map[AD7124_Status].value &
					       CHN_ACTIVE_SELECT_MASK);

			if (channelRead == channel) {
				// channel number of sample matches the channel we want to read
				*pSampleValue = (uint32_t)sample_data;
				acquired = true;
			} else {
				DEBUG_MESSAGE("Error channel mismatch in single conversion mode.  Expected: %d, Actual: %d.\r\n",
					      channel, channelRead);
				errorCode = CHANNEL_MISMATCH;
				break;
			}
		} while (acquired == false);

		// Clear the Enable bit in the CHANNEL_X configuration register
		ad7124_register_map[AD7124_Channel_0 + channel].value &= ~((
					uint32_t)AD7124_CH_MAP_REG_CH_ENABLE);
		if ((errorCode = ad7124_write_register(m_pAd7124,
						       ad7124_register_map[AD7124_Channel_0 + channel])) != SUCCESS) {
			DEBUG_MESSAGE("Error clearing Channel %d Enable bit (%ld).\r\n", channel,
				      errorCode);
		}
	}

	return (errorCode);
}


/**
  * @brief  	acquires one channel sample from ADC
  *
  * @param 		[in] channel to acquire
  *
  * @param 		[in, out] sample value
  *
  * @return     error code 0 indicating success or else failure
  *
  * @details    This is used when ADC is in continuous mode, and all channels
  * 			to be sampled are already enabled.  It loops many times to try and capture
  * 			the request channel, or else times out.
  * 			It requires the STATUS byte to be appended to the ADC sample value to determine
  * 			the channel for the ADC value being read.
  */
int32_t ad7124_temperature::sampleChannelCC(uint8_t channel,
		uint32_t * pSampleValue)
{
	/*
	 * The ADC channel sequencer is not very efficient when we have 2 specific
	 * channels to sample. Sampling just want we need would be more efficient.
	 */
	int32_t		errorCode ;
	bool		acquired = false;
	int32_t		sample_data = 0;

	do {
		/*
		 *  this polls the status register READY bit to determine when conversion is done
		 *  this also ensures the STATUS register value is up to date and contains the
		 *  channel that was sampled as well. No need to read STATUS separately
		 */
		if ((errorCode = ad7124_wait_for_conv_ready(m_pAd7124,
				 adc_conv_max_wait)) != SUCCESS) {
			DEBUG_MESSAGE("Error/Timeout waiting for conversion ready %ld\r\n", errorCode);
			break;
		}

		if ((errorCode = ad7124_read_data(m_pAd7124, &sample_data)) != SUCCESS) {
			DEBUG_MESSAGE("Error reading ADC Data (%ld)\r\n", errorCode);
			break;
		}

		// what channel has been read?
		uint8_t channelRead = (ad7124_register_map[AD7124_Status].value &
				       CHN_ACTIVE_SELECT_MASK);

		if (channelRead == channel) {
			// channel number of sample matches the channel we want to read
			*pSampleValue = (uint32_t)sample_data;
			acquired = true;
		} else {
			if (channelRead >= num_of_ad7124_temp_sensors) {
				DEBUG_MESSAGE("Channel Index in status >= %d", num_of_ad7124_temp_sensors);
				errorCode = CHANNEL_MISMATCH;
				break;
			}
		}
	} while (acquired == false);

	return (errorCode);
}


/**
 * @brief	Converts raw ADC code to resistance
 *
 * @param	[in] Raw ADC code
 *
 * @return	resitance in Ohms
 *
 * @details	This converts the raw ADC code to Ohms for cold junction RTD,
 * 			based on the AD7124 Eval board configuration
 */
float ad7124_temperature::convertRawAdcToResistance(uint32_t rawAdcCode)
{
	return (((rawAdcCode - ad7124_max_ad_cnt) /
		 (rtd_chn_gain * ad7124_max_ad_cnt)) * precisionResistor);
}


/**
 * @brief  	Converts raw ADC code to millivolts
 *
 * @param	[in] Raw ADC code
 *
 * @return	voltage in millivolts
 *
 * @details	This converts the raw ADC code to millivolts for thermocouple channel,
 *          based on the AD7124 Eval board configuration
 */
float ad7124_temperature::convertRawAdcToMilliVolts(uint32_t rawAdcCode)
{
	return (((((float)rawAdcCode - ad7124_max_ad_cnt) / (thermocouple_chn_gain *
			ad7124_max_ad_cnt)) * ad7124_vRef) * 1000);
}


/**
 * @brief	converts raw ADC code to voltage
 *
 * @param	[in] Raw ADC code
 *
 * @return	ADC code in volts
 *
 * @details	This converts the raw ADC code to voltage, based on the ref and Gain of the channel
 */
float ad7124_temperature::convertRawAdcToVoltage(uint32_t rawAdcCode,
		float vRef, uint8_t channelGain)
{
	return ((((float)rawAdcCode / ad7124_max_ad_cnt) - 1) * (vRef / channelGain));
}


/**
 * @brief	Set the Iout excitation
 *
 * @param	[in] excitation value
 *
 * @return	Sensor excitation set error status
 *
 * @details	This sets the excitation current for adc conversion
 */
SENSOR_RESULT ad7124_temperature::setIout0(ioutExcitation option)
{
	// update the register struct with the value to write
	ad7124_register_map[AD7124_IOCon1].value &= (~IO_CTRL1_IOUT0_MASK);
	ad7124_register_map[AD7124_IOCon1].value |= AD7124_IO_CTRL1_REG_IOUT0(option);

	// write the register
	int32_t writeRegisterResult = ad7124_write_register(m_pAd7124,
				      ad7124_register_map[AD7124_IOCon1]);
	if (writeRegisterResult != SUCCESS) {
		return(SENSOR_ERROR_SPI);
	} else {
		return(SENSOR_ERROR_NONE);
	}
}


/**
 * @brief	Calibrate the AD7124 temperature sensor channels
 *
 * @return	calibration result status
 *
 * @details	This internally calibrates the AD7124 channels which are used
 * 			for thermocouple and RTD sensors
 */
SENSOR_RESULT ad7124_temperature::calibrate_temp_sensor_channels()
{
	SENSOR_RESULT eSensorResult = SENSOR_ERROR_NONE;

	// Disable the channels before calibration
	for (uint8_t chn = t_thermocouple; chn < num_of_ad7124_temp_sensors; chn++) {
		ad7124_register_map[AD7124_Channel_0 + chn].value &=
			(~AD7124_CH_MAP_REG_CH_ENABLE);
		ad7124_write_register(m_pAd7124, ad7124_register_map[AD7124_Channel_0 + chn]);
	}

	for (uint8_t chn = t_thermocouple; chn < num_of_ad7124_temp_sensors; chn++) {
		// Enable the current channel
		ad7124_register_map[AD7124_Channel_0 + chn].value |=
			AD7124_CH_MAP_REG_CH_ENABLE;
		if (ad7124_write_register(m_pAd7124,
					  ad7124_register_map[AD7124_Channel_0 + chn]) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}

		// Start full scale internal calibration
		ad7124_register_map[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(
					MODE_SELECT_MASK));
		ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(
					internal_full_scale_cal);

		if (ad7124_write_register(m_pAd7124,
					  ad7124_register_map[AD7124_ADC_Control]) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}

		if ((ad7124_wait_for_conv_ready(m_pAd7124, adc_conv_max_wait)) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}

		// Start zero scale internal calibration
		ad7124_register_map[AD7124_ADC_Control].value &= ~(AD7124_ADC_CTRL_REG_MODE(
					MODE_SELECT_MASK));
		ad7124_register_map[AD7124_ADC_Control].value |= AD7124_ADC_CTRL_REG_MODE(
					internal_zero_scale_cal);

		if (ad7124_write_register(m_pAd7124,
					  ad7124_register_map[AD7124_ADC_Control]) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}

		if ((ad7124_wait_for_conv_ready(m_pAd7124, adc_conv_max_wait)) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}

		// Disable the current channel
		ad7124_register_map[AD7124_Channel_0 + chn].value &=
			(~AD7124_CH_MAP_REG_CH_ENABLE);
		if (ad7124_write_register(m_pAd7124,
					  ad7124_register_map[AD7124_Channel_0 + chn]) != SUCCESS) {
			eSensorResult = SENSOR_ERROR_ADC;
			break;
		}
	}

	// Enable the channels post calibration for continuous conversion mode operation.
	// For single conversion mode operation, only one channel is active at a time
	if (conversion_mode == continuous_conversion_mode) {
		for (uint8_t chn = t_thermocouple; chn < num_of_ad7124_temp_sensors; chn++) {
			ad7124_register_map[AD7124_Channel_0 + chn].value |=
				AD7124_CH_MAP_REG_CH_ENABLE;
			ad7124_write_register(m_pAd7124, ad7124_register_map[AD7124_Channel_0 + chn]);
		}
	}

	return eSensorResult;
}

} /* end of namespace adi_sensor_swpack */



/* Define the methods for AD7124 device configurations */

/**
 * @brief	Constructor for 'ad7124_device_config' class
 *
 * @details	This is a constructor for 'ad7124_device_config' class,
 * 			which initialize the AD7124 device
 */
ad7124_device_config::ad7124_device_config()
{
	/* Create the AD7124 device that is shared by several sensors */
	int32_t initResult = ad7124_setup(&pAd7124_dev, sAd7124_init);

	if (initResult == SUCCESS) {
		ad7124_init_status = true;
	} else {
		DEBUG_MESSAGE("Error initializing AD7124 Data struct\r\n");
	}
}


/**
 * @brief	Get the pointer to AD7124 device instance
 *
 * @return	Pointer to AD7124 device instance
 *
 * @details	This gets the pointer to AD7124 device instance
 */
ad7124_dev *ad7124_device_config::get_ad7124_dev_instance()
{
	return pAd7124_dev;
}


/**
 * @brief	Get the AD7124 device init status
 *
 * @return	AD7124 init status
 *
 * @details	This gets the AD7124 device init status
 */
bool ad7124_device_config::get_ad7124_dev_init_status()
{
	return ad7124_init_status;
}
