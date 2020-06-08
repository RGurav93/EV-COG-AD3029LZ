/*!
 *****************************************************************************
   @file:    main.cpp

   @brief:   Main interface for temperature sampling and data dispatching

   @details: This module provides an interface to sample and dispatch the
   	   	   	 temperature data over either local links or BLE
  -----------------------------------------------------------------------------

 Copyright (c) 2018-19, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <base_sensor/adi_sensor_errors.h>
#include <drivers/gpio/adi_gpio.h>
#include <common/adi_timestamp.h>
#include <axl/adxl362/adi_adxl362.h>

#include "adi_initialize.h"
#include "adi_support.h"

#include "platform_drivers.h"
#include "adi_initsystem.h"
#include "btns_leds.h"

#include "sampling_engine.h"
#include "uart_dispatcher.h"
#include "adi_ble_dispatcher.h"
#include "wakeup_timer.h"
#include "uart_support.h"
#include "main.h"
#include "app_config.h"

#if ACTIVE_DEVICE == ID_AD7124
#include "ad7124_temperature_sensor.h"
#endif

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Select communication mode. Comment below to select UART as default com mode */
#define ADI_BLUETOOTH_COMM

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

using namespace adi_sensor_swpack;

const uint32_t ADXL362_REG_ACT_INACT_CTL_bACT_EN = 0x1;

/* ADXL362 sensitivity value (LSB/g) at +/-2g measurement range */
const float adxl362_sensitivity = 1000.0;

// Sensor Channel IDs
const uint8_t PRIMARY_TEMPERATURE_SENSOR_ID = 1;
const uint8_t ACCELEROMETER_SENSOR_ID = 2;

// pointers to shared objects
ADXL362 				* pAxl = NULL;
SamplingEngine			* pSampleEngine = NULL;
Dispatcher				* pDispatcher = NULL;

/******************************************************************************/
/********************** Function Prototypes ***********************************/
/******************************************************************************/

static void initAxl(void);
static void do_pre_hibernate_entry_operations(void);
static void do_post_hibernate_exit_operations(void);

/******************************************************************************/
/********************** Function Definitions **********************************/
/******************************************************************************/

/*!
 * @brief      Main
 *
 * @details    Application entry point.
 *
 * @param [in] argc : Number of arguments (unused)
 *
 * @param [in] argv : Arguments (unused)
 *
 */
int main(int argc, char *argv[])
{
	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */
	adi_initComponents();

	/* Initialize the system */
	InitSystem();

#if ACTIVE_DEVICE == ID_AD7124
	/* Create AD7124 device configurations */
	ad7124_device_config	ad7124_dev_cfg;
#endif

	// Create the temperature sensors
	ADT7420        			adt7420;
#if ACTIVE_DEVICE == ID_AD7124
	ad7124_temperature	ad7124Temperature(ad7124_dev_cfg.get_ad7124_dev_instance(),
			RTD_PRECISION_RESISTANCE);
#endif

	// Create the ADXL362 Accelerometer
	ADXL362 				Axl;
	pAxl = &Axl;

	// Create the Sampling Engine to gather the samples
	SamplingEngine			sampleEngine;
	pSampleEngine = &sampleEngine;

	// Create the BLE or UART based dispatcher, but not both
#if defined ADI_BLUETOOTH_COMM
	AdiBleDispatcher 	Dispatcher;
#else
	UartDispatcher 		Dispatcher;
#endif

	pDispatcher = & Dispatcher;

	// Result value for all sensor method
	SENSOR_RESULT			eSensorResult;

	// Add all the temperature sensors to the sampleEngine
#if ACTIVE_DEVICE == ID_AD7124
	if (ad7124_dev_cfg.get_ad7124_dev_init_status() == true) {
		if ((eSensorResult = sampleEngine.addSensor(&ad7124Temperature)) !=
		    SENSOR_ERROR_NONE) {
			PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
		}
	}
#endif

	if ((eSensorResult = sampleEngine.addSensor(&adt7420)) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}

	initAxl();

	ADI_RTC_RESULT eRtc  = wakeup_run(WAKEUP_TIMER_INTERVAL);
	DEBUG_RESULT("wakeup_run failed", eRtc, ADI_RTC_SUCCESS);

	while (1) {
		/*
		 * This puts the processor into hibernation mode, waiting for interrupts
		 * The following interrupts can can wake the processor
		 * 		BTN1 - user initiates sample
		 * 		Axl  - acceleration threshold exceeded, triggers a Sample and transmit data
		 *   	RTC  - Sample and transmit data on a periodic basic
		 *
		 * Before entering into hibernate mode, all the used peripherals must be
		 * disable. Once the hibernate mode is exited by one of the above mentioned
		 * interrupts, the peripherals which were disabled before, must be enabled again.
		 *
		 * In addition to that, when device exits from hibernate mode, by default
		 * only the Bank0/Bank1 of data SRAM is retained. Therefore it is required
		 * to map .data and .bss sections of memory to Bank0/1 of SDRAM in the device
		 * linker file of the project (ADuCM3029.ld), so that data is retained when
		 * controller comes out of hibernate mode.
		 */

		/* Perform the operations needed before entering into hibernate mode */
		do_pre_hibernate_entry_operations();

		/* enter full hibernate mode with no wakeup flag (always go back to sleep) and no masking */
		if (adi_pwr_EnterLowPowerMode(ADI_PWR_MODE_HIBERNATE, &iHibernateExitFlag, 0)) {
			DEBUG_MESSAGE("System Entering to Low Power Mode failed");
		}

		/* Perform the operations needed after exit from hibernate mode */
		do_post_hibernate_exit_operations();

		// Give the dispatcher some time to do any work it needs.
		pDispatcher->Idle(250);

		sampleTransmitCycle();
	}

	// Code should never get this far
	Trap();
}


/*!
 * @brief      Samples all enabled sensors, and transmits them with the dispatcher
 *
 * @param	   None
 *
 * @details
 */
void sampleTransmitCycle(void)
{
	SENSOR_RESULT 			eSensorResult;
	DISPATCHER_ERROR_TYPE 	eDispatcherResult;
	uint32_t				nTime = 0ul;

	if (pDispatcher->isConnected()) {

		/* Print the current RTC time post awake */
		rtc_print_time();

		/* Get time stamp for temperature */
		nTime = GET_TIME();

		/* Get temperature in Celsius */
		float nTempCel;
		eSensorResult = pSampleEngine->getTemperatureSample(&nTempCel);

		// dispatch temperature sample here if all is well
		if (eSensorResult == SENSOR_ERROR_NONE) {
			if ((eDispatcherResult = pDispatcher->TransmitTemperature(nTempCel, nTime,
						 PRIMARY_TEMPERATURE_SENSOR_ID)) != DISPATCHER_ERROR_NONE) {
				DEBUG_MESSAGE("Unable to transmit temperature data, Error %d",
					      eDispatcherResult);
			}
		}

		/* Get time stamp for axl*/
		nTime = GET_TIME();

		// Now get the axl data
		int16_t x, y, z;

		eSensorResult = pAxl->getX((uint8_t*)&x, 2u);
		if (eSensorResult == SENSOR_ERROR_NONE) {
			eSensorResult = pAxl->getY((uint8_t*)&y, 2u);
			if (eSensorResult == SENSOR_ERROR_NONE) {
				pAxl->getZ((uint8_t*)&z, 2u);
			}
		}

		// dispatch acceleration sample here if all is well
		if (eSensorResult == SENSOR_ERROR_NONE) {
			if ((eDispatcherResult = pDispatcher->TransmitAxlXYZ((float)
						 x/adxl362_sensitivity,
						 (float)y/adxl362_sensitivity, (float)z/adxl362_sensitivity,
						 nTime, ACCELEROMETER_SENSOR_ID)) != DISPATCHER_ERROR_NONE) {
				DEBUG_MESSAGE("Unable to transmit temperature data, Error %d",
					      eDispatcherResult);
			}
		}

		// toggle LED to show sampling
		ADI_GPIO_RESULT eToggleResult;
		if(ADI_GPIO_SUCCESS != (eToggleResult = adi_gpio_Toggle(LED1_PORT_NUM,
							LED1_PIN_NUM))) {
			DEBUG_MESSAGE("adi_gpio_Toggle failed\n");
		}
	}

	/*
	 * Read Axl Status register to clear any activity interrupts
	 * otherwise only a single WAKE" interrupt is detected, until a different Interrupt occurs
	 */
	uint8_t dummyBuffer;
	if (pAxl->readRegister(ADXL362::REG_STATUS, &dummyBuffer,
			       1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}
}


/*!
 * @brief      performs accelerometer initialization and configuration
 *
 * @param	   None
 *
 * @details
 */
static void initAxl(void)
{
	SENSOR_RESULT eSensorResult = SENSOR_ERROR_NONE;

	/* Open Accelerometer */
	if ((eSensorResult = pAxl->open()) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
		Trap();
	}

	// Configure ADXL326 activity Interrupt output pin #1
	if (pAxl->configureInterruptPolarity(true,
					     ADXL362::INTERRUPT_PIN_1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}
	if (pAxl->configureInterruptSources(true, ADXL362::INTERRUPT_PIN_1,
					    INTERRUPT_SOURCE_ACT ) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}

	// Configure ADXL362 Wakeup interrupt to MCU on Activity above threshold
	// Set activity threshold to +/- 1800 mg (= 0x708) counts, just over +/- 1g.
	// (*Default/reset sensitivity is +/-2g)
	if (pAxl->writeRegister(ADXL362::REG_THRESH_ACT_H, 0x07,
				1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}
	if (pAxl->writeRegister(ADXL362::REG_THRESH_ACT_L, 0x08,
				1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}

	// Enable Activity detection write REG_ACT_INACT_CTL bit ACT_EN to 1 to enable activity detection
	if (pAxl->writeRegister(ADXL362::REG_ACT_INACT_CTL,
				ADXL362_REG_ACT_INACT_CTL_bACT_EN, 1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}

	// Enable ADXL WakeUP measurement mode
	if (pAxl->writeRegister(ADXL362::REG_POWER_CTL, 0x0A, 1) != SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
	}

	/* Start measurement */
	if ((eSensorResult = pAxl->start())!= SENSOR_ERROR_NONE) {
		PRINT_SENSOR_ERROR(DEBUG_MESSAGE, eSensorResult);
		Trap();
	}
}


/*!
 * @brief      handles button 2 being pressed by selecting next sensor on sampleEngine
 *
 * @param	   None
 *
 * @details
 */
void handleButton2(void)
{
	// Button 2 is used to change the temperature channel being sampled
	pSampleEngine->selectNextSensor();

	/* toggle LED 2  to indicate the change has been made*/
	ADI_GPIO_RESULT eToggleResult = adi_gpio_Toggle(LED2_PORT_NUM, LED2_PIN_NUM);
	DEBUG_RESULT("Error toggling LED2.", eToggleResult, ADI_GPIO_SUCCESS);
}


/*!
 * @brief	Perform the Pre-Hibernate entry operations
 *
 * @return  void
 *
 * @details	This function disables all the used peripherals
 * 			before entering into hibernate power down mode
 */
static void do_pre_hibernate_entry_operations(void)
{
	/* Disable DMA controller */
	pADI_DMA0->CFG &= ~BITM_DMA_CFG_MEN;

	/* Disable UART0 */
	pADI_UART0->DIV = 0;

	/* Disable SPI2 (used for BLE) */
	pADI_SPI2->CTL &= ~BITM_SPI_CTL_SPIEN;

	/* Disable SPI1 (used for ADXL362) */
	pADI_SPI1->CTL &= ~BITM_SPI_CTL_SPIEN;

	/* Disable SPI0 (used for Sigma Delta ADC interface) */
	pADI_SPI0->CTL &= ~BITM_SPI_CTL_SPIEN;
}


/*!
 * @brief	Perform the Post-Hibernate exit operations
 *
 * @return  void
 *
 * @details	This function enables all the used peripherals
 * 			after exiting from the hibernate power down mode
 */
static void do_post_hibernate_exit_operations(void)
{
	/* Enable the DMA Controller */
	pADI_DMA0->CFG |= BITM_DMA_CFG_MEN;

	/* Enable UART0 */
	pADI_UART0->DIV = UART0_CFG_DIVC;

	/* Enable SPI2 (used for BLE) */
	pADI_SPI2->CTL |= BITM_SPI_CTL_SPIEN;

	/* Enable the SPI1 (used for ADXL362) */
	pADI_SPI1->CTL |= BITM_SPI_CTL_SPIEN;

	/* Enable SPI0 (used for Sigma Delta ADC interface) */
	pADI_SPI0->CTL |= BITM_SPI_CTL_SPIEN;
}
