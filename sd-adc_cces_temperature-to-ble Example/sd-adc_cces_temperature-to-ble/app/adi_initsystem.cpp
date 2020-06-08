/*!
 *****************************************************************************
  @file:  adi_initsystem.cpp

  @brief: Initialize the system peripherals

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

#include <adi_processor.h>
#include <drivers/dma/adi_dma.h>
#include <common/adi_timestamp.h>
#include <drivers/gpio/adi_gpio.h>
#include <drivers/xint/adi_xint.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/rtc/adi_rtc.h>

#include "uart_support.h"
#include "platform_drivers.h"
#include "sampling_engine.h"
#include "btns_leds.h"
#include "wakeup_timer.h"
#include "adi_initsystem.h"
#include "main.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/* Used by the GPIO pin control to store config settings*/
static uint8_t aGpioMemory[ADI_GPIO_MEMORY_SIZE];

/* Used by the External Interrupt pin control to store config settings*/
static uint8_t xintMemory[ADI_XINT_MEMORY_SIZE];

/******************************************************************************/
/********************** Function Definitions **********************************/
/******************************************************************************/

// static Call back for counting interrupts on XINT0_WAKE2
static void xintWake2Callback(void* pCBParam, uint32_t nEvent,
			      void* pEventData)
{
	/* ADXL362 interrupt */
	if ((ADI_XINT_EVENT) nEvent == ADI_XINT_EVENT_INT2) {
		adi_pwr_ExitLowPowerMode(&iHibernateExitFlag);
	}
}


// static Call back for interrupts on XINT0_WAKE1
static void xintWake1Callback(void* pCBParam, uint32_t nEvent,
			      void* pEventData)
{
	/* Push button 1 */
	if ((ADI_XINT_EVENT) nEvent == ADI_XINT_EVENT_INT1) {
		adi_pwr_ExitLowPowerMode(&iHibernateExitFlag);
	}
}


// GPIO event Callback function
// *Note: This interrupt does not get triggered when controller is in hibernate
// 		  mode. In order to select the next sensor, press bush button 2 when
// 		  controller is awake (e.g. After PB1, PB2 can be pressed to select next sensor)
static void pinIntCallback(void* pCBParam, uint32_t Port,  void* PinIntData)
{
	/* Push button 2 */
	if ((Port == (uint32_t)PB2_PORT_NUM)
	    && (*(uint32_t*)PinIntData & PB2_PIN_NUM)) {
		handleButton2();
	}
}



/*!
 * @brief      Initializes the system
 *
 * @details    This function is responsible for initializing the power service, GPIOs, interrupts
 *             and bluetooth subsystem. It also initializes the realtime clock for to timestamp
 *             the outgoing sensor data packets.
 */
void InitSystem(void)
{
	ADI_PWR_RESULT  ePwr;
	ADI_XINT_RESULT xintResult;

	/* Explicitly disable the watchdog timer */
	*pREG_WDT0_CTL = 0x0u;

	/* Set up the DMA Controller to transmit UART messages */
	adi_dma_Init();

	/* Initialize the UART link to dispatch debug messages and sensor data */
	uart_init();

	/* Initialize clocks */
	ePwr = adi_pwr_Init();
	DEBUG_RESULT("Error initializing the power service.\r\n", ePwr,
		     ADI_PWR_SUCCESS);

	// This configures the low power clock mux and enables the source
	ePwr = adi_pwr_SetLFClockMux(ADI_CLOCK_MUX_LFCLK_LFXTAL);
	DEBUG_RESULT("Error configuring the LFClockMux.\r\n", ePwr, ADI_PWR_SUCCESS);

	ePwr = adi_pwr_EnableClockSource(ADI_CLOCK_SOURCE_LFXTAL,true);
	DEBUG_RESULT("Error enabling the LFXTAL source.\r\n", ePwr, ADI_PWR_SUCCESS);

	// Configure the core and peripheral clocks
	ePwr = adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1u);
	DEBUG_RESULT("Error configuring the core clock.\r\n", ePwr, ADI_PWR_SUCCESS);

	ePwr = adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1u);
	DEBUG_RESULT("Error configuring the peripheral clock.\r\n", ePwr,
		     ADI_PWR_SUCCESS);

	// This inits the ADI GPIO functions, primarily with a block of memory used by callbacks
	ADI_GPIO_RESULT eGpioResult = adi_gpio_Init(aGpioMemory, sizeof(aGpioMemory));
	DEBUG_RESULT("adi_gpio_Init", eGpioResult, ADI_GPIO_SUCCESS);


	// Initialize the push button IOs and Interrupts and set GPIO input
	// DEBUG_MESSAGE as it does not exit program, only prints error message
	do {
		/* set GPIO input */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_InputEnable(PB2_PORT_NUM,
						      PB2_PIN_NUM, true))) {
			DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
			break;
		}

		/* set Pin polarity as rising edge */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_SetGroupInterruptPolarity(
				PB2_PORT_NUM, PB2_PIN_NUM))) {
			DEBUG_MESSAGE("adi_gpio_SetGroupInterruptPolarity failed\n");
			break;
		}

		/* Enable pin interrupt on group interrupt A */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_SetGroupInterruptPins(
				PB2_PORT_NUM, ADI_GPIO_INTA_IRQ, PB2_PIN_NUM))) {
			DEBUG_MESSAGE("adi_gpio_SetGroupInterruptPins failed\n");
			break;
		}

		/* Register the callback */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_RegisterCallback (
				ADI_GPIO_INTA_IRQ, pinIntCallback, (void*)ADI_GPIO_INTA_IRQ))) {
			DEBUG_MESSAGE("adi_gpio_RegisterCallback failed\n");
			break;
		}

		/* set GPIO output LED 1 and 2 */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_OutputEnable(LED1_PORT_NUM,
						      LED1_PIN_NUM, true))) {
			DEBUG_MESSAGE("adi_gpio_SetDirection failed\n");
			break;
		}

		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_OutputEnable(LED2_PORT_NUM,
						      LED2_PIN_NUM, true))) {
			DEBUG_MESSAGE("adi_gpio_SetDirection failed\n");
			break;
		}
	} while (0);


	/* Initialize the XINT driver */
	xintResult = adi_xint_Init(xintMemory, ADI_XINT_MEMORY_SIZE);
	DEBUG_RESULT("adi_xint_Init failed", xintResult, ADI_XINT_SUCCESS);

	// Initialize the External Interrupts handlers
	// DEBUG_MESSAGE as it does not exit program, only prints error message
	do {
		/* Configure the pin for the ADXL362 Interrupt to act as an input */
		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_InputEnable(ADI_GPIO_PORT0,
						      ADI_GPIO_PIN_13, true))) {
			DEBUG_MESSAGE("adi_gpio_InputEnable failed port 0\n");
			break;
		}

		/*
		  *  Set the configuration mode bits for GPIO13/WAKE2 [27:26] to 2'b00
		  *  The default mode set in the GPIO_CFG-Pin13 config bits seems to be 2'b01
		  *  which this pin doesn't support
		  *  This setting is not done in the adi_gpio API calls
		  */
		pADI_GPIO0->CFG &= 0xF3FFFFFF;

		if(ADI_GPIO_SUCCESS != (eGpioResult = adi_gpio_InputEnable(PB1_PORT_NUM,
						      PB1_PIN_NUM, true))) {
			DEBUG_MESSAGE("adi_gpio_InputEnable failed\n");
			break;
		}

		/* Register the callback for XINT1 external interrupts  for BTN1 */
		if(ADI_XINT_SUCCESS != (xintResult = adi_xint_RegisterCallback (
				ADI_XINT_EVENT_INT1, xintWake1Callback, NULL))) {
			DEBUG_MESSAGE("adi_xint_RegisterCallback failed\n");
			break;
		}

		/* Register the callback for XINT2 external interrupts on WAKE2 for ADXL362 */
		if(ADI_XINT_SUCCESS != (xintResult = adi_xint_RegisterCallback (
				ADI_XINT_EVENT_INT2, xintWake2Callback, NULL))) {
			DEBUG_MESSAGE("adi_xint_RegisterCallback failed\n");
			break;
		}

		/* Enable XINT0 for falling edge interrupt on BTN1*/
		if(ADI_XINT_SUCCESS != (xintResult = adi_xint_EnableIRQ (ADI_XINT_EVENT_INT1,
						     ADI_XINT_IRQ_FALLING_EDGE))) {
			DEBUG_MESSAGE("adi_xint_EnableExIRQ failed\n");
			break;
		}
		/* Enable XINT2 for falling edge interrupt on ADXL362 WAKE2 input */
		if(ADI_XINT_SUCCESS != (xintResult = adi_xint_EnableIRQ (ADI_XINT_EVENT_INT2,
						     ADI_XINT_IRQ_FALLING_EDGE))) {
			DEBUG_MESSAGE("adi_xint_EnableExIRQ failed\n");
			break;
		}

	} while (0);

#ifdef __EVCOG__
	/* Unlike the CUP board, the COG board I2C pins are pulled to a GPIO line instead of VDD */
	eGpioResult = adi_gpio_OutputEnable(ADI_GPIO_PORT1, ADI_GPIO_PIN_12, true);
	DEBUG_RESULT("adi_gpio_OutputEnable", eGpioResult, ADI_GPIO_SUCCESS);
	eGpioResult = adi_gpio_SetHigh(ADI_GPIO_PORT1, ADI_GPIO_PIN_12);
	DEBUG_RESULT("adi_gpio_SetHigh", eGpioResult, ADI_GPIO_SUCCESS);
#endif

	DEBUG_MESSAGE("Starting Temperature-BLE demo application\r\n");

	/* Init timestamping */
	INIT_TIME();

	ADI_RTC_RESULT  eRtc;
	// configure the RTC wake-up alarms
	eRtc = wakeup_initialize();
	DEBUG_RESULT("Error initializing the wake-up RTC alarm", eRtc,
		     ADI_RTC_SUCCESS );
}
