
/*!
 *****************************************************************************
  @file:  ad7124_temperature_sensor.h

  @brief: Headers for temperature sensing interface for AD7124 device

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2018, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef AD7124_TEMPERATURE_SENSOR_H_
#define AD7124_TEMPERATURE_SENSOR_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <temp/adi_temperature.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "app_config.h"
#include "ad7124.h"

#ifdef __cplusplus
}
#endif

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Default RTD precision resistance (Rref) value for AD7124 (22K) */
#define DEFAULT_AD7124_RTD_RREF_VALUE	22000.0

/******************************************************************************/
/********************** Class Declaration *************************************/
/******************************************************************************/

namespace adi_sensor_swpack
{
/**
 * @class AD7124 Temperature Class
 *
 * @brief AD7124 temperature class interface.
 *
 **/
class ad7124_temperature : public Temperature
{
public:

	enum ioutExcitation {
		Iout_off = 0x0,
		Iout_50uA = 0x1,
		Iout_100uA = 0x2,
		Iout_250uA = 0x3,
		Iout_500uA = 0x4,
		Iout_100nA = 0x7
	};

	/* Constructor */
	ad7124_temperature(ad7124_dev * pAd7124_dev,
			   float rtd_rref=DEFAULT_AD7124_RTD_RREF_VALUE,
			   ioutExcitation rtd_excitation_on_current=Iout_100uA,
			   ioutExcitation rtd_excitation_off_current=Iout_100nA);

	/*!< Pure virtual functions must be implemented by the derived class */
	virtual SENSOR_RESULT open();
	virtual SENSOR_RESULT start();
	virtual SENSOR_RESULT stop();
	virtual SENSOR_RESULT close();
	virtual SENSOR_RESULT getTemperature(uint8_t *pTemperature,
					     const uint32_t sizeInBytes);
	virtual SENSOR_RESULT getTemperatureInFahrenheit(float *pTemperatureFahrenheit);
	virtual SENSOR_RESULT getTemperatureInCelsius(float *pTemperatureCelsius);

private:
	const uint16_t adc_conv_max_wait =
		10000;	/* Max wait for adc conversion to over */
	const float ad7124_vRef = 2.5;		/* Default Vref voltage for AD7124 */
	const float ad7124_resoln = 24.0;	/* AD7124 resolution in bits */
	const float ad7124_max_ad_cnt =
		8388608.0;	/* AD7124 full-scale value (2^(24-1)) */
	const float thermocouple_chn_gain =
		128.0;	/* Gain for thermocouple input channel */
	const float rtd_chn_gain = 8.0;				/* Gain for RTD input channel */

	float precisionResistor;					/* RTD reference resistance */
	ioutExcitation
	rtd_excitation_on_current;	/* RTD excitation trigger/on current */
	ioutExcitation rtd_excitation_off_current;	/* RTD excitation off current */

	int32_t getNextSample(float * temperatureValue);
	float convertRawAdcToVoltage(uint32_t rawAdcCode, float vRef,
				     uint8_t channelGain);
	float convertRawAdcToResistance(uint32_t rawAdcCode);
	float convertRawAdcToMilliVolts(uint32_t rawAdcCode);
	int32_t sampleChannelSC(uint8_t channel, uint32_t * sampleValue);
	int32_t sampleChannelCC(uint8_t channel, uint32_t * sampleValue);
	SENSOR_RESULT calibrate_temp_sensor_channels();

	/**
		 * Define the AD7124 conversion modes
		 */
	enum adc_converion_modes {
		continuous_conversion_mode = 0,
		single_conversion_mode = 1,
		standby_mode = 2,
		power_down_mode = 3,
		internal_zero_scale_cal = 5,
		internal_full_scale_cal = 6,
	};

	/**
	 * Define the temperature sensors connected to AD724 Eval for temp sensing
	 * 1) T type Thremocouple (Channel 0)
	 * 2) Cold junction compensation RTD (Channel 1)
	 */
	enum ad7124_temperature_sensors {
		t_thermocouple = 0,
		cjc_rtd,
		num_of_ad7124_temp_sensors
	};

#define	MODE_SELECT_MASK		0x0F
#define	CHN_ACTIVE_SELECT_MASK	0x0F

	/* Used to set the excitation currents for low/normal power operation */
#define IO_CTRL1_IOUT0_MASK 	0x000700

	SENSOR_RESULT setIout0(ioutExcitation option);

	/* Conversion mode for ADC */
	adc_converion_modes conversion_mode;

	// device member properties for AD7124
	ad7124_dev			*m_pAd7124 = NULL;
};
}


/**
 * @class AD7124 device configuration
 *
 * @brief This class defines the interfaces for AD7124 device configuration
 *
 **/
class ad7124_device_config
{
public:
	/* Constructor */
	ad7124_device_config();

	/* Get the AD7124 device instance */
	ad7124_dev *get_ad7124_dev_instance();

	/* Get the AD7124 device init status */
	bool get_ad7124_dev_init_status();

private:
	/* AD7124 device init structure pointer */
	ad7124_dev *pAd7124_dev = NULL;

	/* AD7124 device init status */
	bool ad7124_init_status = false;
};

#endif /* AD7124_TEMPERATURE_SENSOR_H_ */
