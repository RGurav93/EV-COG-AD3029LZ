/*!
 *****************************************************************************
  @file:  adi_ble_dispatcher.cpp

  @brief: This class transmits samples over BLE

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

#include <common/adi_timestamp.h>
#include <common/adi_error_handling.h>

#include <base_sensor/adi_sensor_errors.h>

#include <adi_ble_config.h>
#include <drivers/pwr/adi_pwr.h>
#include <radio/adi_ble_radio.h>
#include <framework/noos/adi_ble_noos.h>

#include "app_config.h"
#include "uart_support.h"
#include "sampling_engine.h"
#include "adi_ble_dispatcher.h"
#include "adi_support.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Peripheral advertisement mode */
#define PERIPHERAL_ADV_MODE      ((ADI_BLE_GAP_MODE)(ADI_BLE_GAP_MODE_CONNECTABLE | ADI_BLE_GAP_MODE_DISCOVERABLE))

/* BLE event processing dispatcher timeout. Waits for 2sec for BLE events */
#define ADI_APP_DISPATCH_TIMEOUT (2000)

/* Form a sensor id and packet type header byte
 * (Sensor ID: 8:2, Packet Type: 1:0) */
#define BLE_SENSOR_ID_PCKT_TYPE(id, packet_type) ((id << 2u) | (packet_type & 0x3))

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

using namespace adi_sensor_swpack;

static bool               gbConnected;
static ADI_BLE_GAP_MODE   geMode;
static bool				  registration_status = false;
static const uint8_t 	  PRIMARY_TEMPERATURE_SENSOR_ID = 1;
static const uint8_t 	  ACCELEROMETER_SENSOR_ID = 2;

/* Initialize temperature sensor registration packet structure */
AdiBleDispatcher::registration_packet temp_sensor_reg_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(PRIMARY_TEMPERATURE_SENSOR_ID, REGISTRATION_PACKET_ID),
	.num_fields = 1,
	.data_type = PACKET_DATA_TYPE_FLOAT,
	"Temperature"
};

/* Initialize temperature sensor field packet structure */
AdiBleDispatcher::field_name_packet temp_sensor_field_name_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(PRIMARY_TEMPERATURE_SENSOR_ID, FIELD_NAME_PACKET_ID),
	.field_id = 0,
	"Celsius"
};

/* Initialize temperature sensor data packet structure */
AdiBleDispatcher::data_packet temp_sensor_data_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(PRIMARY_TEMPERATURE_SENSOR_ID, DATA_PACKET_ID),
	.sensor_data1 = {0,0,0,0},
	.sensor_data2 = {0,0,0,0},
	.sensor_data3 = {0,0,0,0},
	.sensor_data4 = {0,0,0,0},
};


/* Initialize ADXL362 sensor registration packet structure */
AdiBleDispatcher::registration_packet axl362_sensor_reg_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(ACCELEROMETER_SENSOR_ID, REGISTRATION_PACKET_ID),
	.num_fields = 3,
	.data_type = PACKET_DATA_TYPE_FLOAT,
	"ADXL362",
};

/* Initialize ADXL362 sensor field1 packet structure */
AdiBleDispatcher::field_name_packet axl362_sensor_field1_name_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(ACCELEROMETER_SENSOR_ID, FIELD_NAME_PACKET_ID),
	.field_id = 0,
	"X-axis [G]"
};

/* Initialize ADXL362 sensor field2 packet structure */
AdiBleDispatcher::field_name_packet axl362_sensor_field2_name_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(ACCELEROMETER_SENSOR_ID, FIELD_NAME_PACKET_ID),
	.field_id = 1,
	"Y-axis [G]"
};

/* Initialize ADXL362 sensor field3 packet structure */
AdiBleDispatcher::field_name_packet axl362_sensor_field3_name_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(ACCELEROMETER_SENSOR_ID, FIELD_NAME_PACKET_ID),
	.field_id = 2,
	"Z-axis [G]"
};

/* Initialize ADXL362 sensor data packet structure */
AdiBleDispatcher::data_packet axl362_sensor_data_packet = {
	.pkt_type_sensor_id = BLE_SENSOR_ID_PCKT_TYPE(ACCELEROMETER_SENSOR_ID, DATA_PACKET_ID),
	.sensor_data1 = {0,0,0,0},
	.sensor_data2 = {0,0,0,0},
	.sensor_data3 = {0,0,0,0},
	.sensor_data4 = {0,0,0,0},
};

// forward declaration of callback function
static void ApplicationCallback(void * pCBParam, uint32_t Event, void * pArg);

/******************************************************************************/
/********************** Method Definitions ************************************/
/******************************************************************************/

/*!
 * @brief      Bluetooth Temperature sensor dispatcher Constructor
 *
 * @param  	   None
 *
 * @details    creates and initializes an ADI BLE Dispatcher
 */
AdiBleDispatcher::AdiBleDispatcher()
{
	/* Initialize Bluetooth */
	InitBluetoothLowEnergy();
}


/*!
 * @brief      Event Idle loop
 *
 * @param  	   nTimeoutMilliSeconds Time to idle waiting for events in milliseconds
 *
 * @details    Idles, waiting for an event to happen on the BLE link.
 */
void AdiBleDispatcher::Idle(const uint16_t nTimeoutMilliSeconds)
{
	ADI_BLER_RESULT     eResult;
	ADI_BLER_CONN_INFO  sConnInfo;

	/* Dispatch events nTimeoutMilliSeconds - they will arrive in the application callback */
	eResult = adi_ble_DispatchEvents(nTimeoutMilliSeconds);
	DEBUG_RESULT("Error dispatching events to the callback.\r\n", eResult,
		     ADI_BLER_SUCCESS);

	/* If not connected, may need to set advertising mode */
	if (gbConnected == false) {
		if (geMode != PERIPHERAL_ADV_MODE) {
			SetAdvertisingMode();
		}
	} else {
		if (registration_status == false) {
			/* Perform the sensor registration when the device connection is established */

			adi_ble_GetConnectionInfo(&sConnInfo);

			/* Register the temperature sensor and fields */
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(temp_sensor_reg_packet),(uint8_t*)&temp_sensor_reg_packet);
			DEBUG_RESULT("Error registering temperature sensor.\r\n", eResult,
				     ADI_BLER_SUCCESS);

			delay_ms(50);
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(temp_sensor_field_name_packet),(uint8_t*)&temp_sensor_field_name_packet);
			DEBUG_RESULT("Error registering temperature sensor field.\r\n", eResult,
				     ADI_BLER_SUCCESS);

			/* Register the ADXL362 sensor and fields */
			delay_ms(50);
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(axl362_sensor_reg_packet),(uint8_t*)&axl362_sensor_reg_packet);
			DEBUG_RESULT("Error registering ADXL362 sensor.\r\n", eResult,
				     ADI_BLER_SUCCESS);

			delay_ms(50);
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(axl362_sensor_field1_name_packet),
							(uint8_t*)&axl362_sensor_field1_name_packet);
			DEBUG_RESULT("Error registering ADXL362 sensor field1.\r\n", eResult,
				     ADI_BLER_SUCCESS);

			delay_ms(50);
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(axl362_sensor_field2_name_packet),
							(uint8_t*)&axl362_sensor_field2_name_packet);
			DEBUG_RESULT("Error registering ADXL362 sensor field2.\r\n", eResult,
				     ADI_BLER_SUCCESS);

			delay_ms(50);
			eResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
							sizeof(axl362_sensor_field3_name_packet),
							(uint8_t*)&axl362_sensor_field3_name_packet);
			DEBUG_RESULT("Error registering ADXL362 sensor field3.\r\n", eResult,
				     ADI_BLER_SUCCESS);
		}
	}
}


/*!
 * @brief      is the dispatcher connected to a receiver
 *
 * @param  	   None
 *
 * @details    returns true if BLE link is connected
 */
bool AdiBleDispatcher::isConnected(void)
{
	// for the BLE, can simple return value of global
	return (gbConnected);
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
DISPATCHER_ERROR_TYPE AdiBleDispatcher::TransmitTemperature(
	const float temperatureSample, const uint32_t timeStamp, const uint8_t sensorID)
{
	DISPATCHER_ERROR_TYPE eResult = DISPATCHER_ERROR_NONE;
	ADI_BLER_RESULT     eBleResult;
	ADI_BLER_CONN_INFO  sConnInfo;

	/* If connected, send data */
	if (gbConnected == true) {
		adi_ble_GetConnectionInfo(&sConnInfo);

		DEBUG_MESSAGE("BLE Sensor ID: %d Temperature: %05.2f C" EOL, sensorID,
			      temperatureSample);

		temp_sensor_data_packet.sensor_data1.fValue = temperatureSample;

		eBleResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
						   sizeof(temp_sensor_data_packet),(uint8_t*)&temp_sensor_data_packet);
		if (eBleResult != ADI_BLER_SUCCESS) {
			DEBUG_MESSAGE("Error sending the data.\r\n");
			eResult  = DISPATCHER_ERROR_TRANSPORT;
		}

		delay_ms(10);
	}

	return (eResult);
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
DISPATCHER_ERROR_TYPE AdiBleDispatcher::TransmitAxlXYZ(const float axlX,
		const float axlY, const float axlZ,
		const uint32_t timeStamp, const uint8_t sensorID)
{
	DISPATCHER_ERROR_TYPE 	eResult = DISPATCHER_ERROR_NONE;
	ADI_BLER_RESULT     	eBleResult;
	ADI_BLER_CONN_INFO  	sConnInfo;

	/* If connected, send data */
	if (gbConnected == true) {
		adi_ble_GetConnectionInfo(&sConnInfo);

		DEBUG_MESSAGE("BLE Sensor ID: %d Acceleration X: %5.2f G  Y: %5.2f G  Z: %5.2f G"
			      EOL,
			      sensorID, axlX, axlY, axlZ);

		axl362_sensor_data_packet.sensor_data1.fValue = axlX;
		axl362_sensor_data_packet.sensor_data2.fValue = axlY;
		axl362_sensor_data_packet.sensor_data3.fValue = axlZ;

		eBleResult = adi_radio_DE_SendData(sConnInfo.nConnHandle,
						   sizeof(axl362_sensor_data_packet),(uint8_t *)&axl362_sensor_data_packet);
		if (eBleResult != ADI_BLER_SUCCESS) {
			DEBUG_MESSAGE("Error sending the data.\r\n");
			eResult  = DISPATCHER_ERROR_TRANSPORT;
		}

		delay_ms(10);
	}

	return (eResult);
}


/*!
 * @brief      Set Advertising Mode
 *
 * @details    Helper function to avoid repeated code in main.
 *
 */
void AdiBleDispatcher::SetAdvertisingMode(void)
{
	ADI_BLER_RESULT eResult;

	eResult = adi_radio_SetMode(PERIPHERAL_ADV_MODE, 0u, 0u);
	DEBUG_RESULT("Error setting the mode.\r\n", eResult, ADI_BLER_SUCCESS);

	eResult = adi_ble_WaitForEventWithTimeout(GAP_EVENT_MODE_CHANGE, 5000u);
	DEBUG_RESULT("Error waiting for GAP_EVENT_MODE_CHANGE.\r\n", eResult,
		     ADI_BLER_SUCCESS);

	eResult = adi_radio_GetMode(&geMode);
	DEBUG_RESULT("Error getting the mode.\r\n", eResult, ADI_BLER_SUCCESS);

	if (geMode != PERIPHERAL_ADV_MODE) {
		DEBUG_MESSAGE("Error in SetAdvertisingMode.\r\n");
	}
}


/*!
 * @brief      Initializes the bluetooth
 *
 * @details    Data Exchange profile is initialized to send
 *             data to the connected central device.
 */
void AdiBleDispatcher::InitBluetoothLowEnergy(void)
{
	ADI_BLER_RESULT eResult;
	uint8_t *       aDataExchangeName = (unsigned char *) "Temperature-BLE Demo";

	/* Initialize radio and framework layer */
	eResult = adi_ble_Init(ApplicationCallback, NULL);
	DEBUG_RESULT("Error initializing the radio.\r\n", eResult, ADI_BLER_SUCCESS);

	/* Configure radio */
	eResult = adi_radio_RegisterDevice(ADI_BLE_ROLE_PERIPHERAL);
	DEBUG_RESULT("Error registering the radio.\r\n", eResult, ADI_BLER_SUCCESS);

	eResult = adi_radio_SetLocalBluetoothDevName(aDataExchangeName,
			strlen((const char *) aDataExchangeName), 0u, 0u);
	DEBUG_RESULT("Error setting local device name.\r\n", eResult, ADI_BLER_SUCCESS);

	SetAdvertisingMode();

	/* Initialize data exchange profile */
	eResult = adi_radio_Register_DataExchangeServer();
	DEBUG_RESULT("Error registering data exchange server.\r\n", eResult,
		     ADI_BLER_SUCCESS);

	/* Now enter infinite loop waiting for connection and then data exchange events */
	DEBUG_MESSAGE("Waiting for connection. Initiate connection on central device please.\r\n");
}


/*!
 * @brief      Application Callback
 *
 * @details    Called by the framework layer (adi_ble_noos.c) when an event occurs that the application did NOT
 *             Explicitly expect by calling #adi_ble_WaitForEventWithTimeout.
 *
 * @param [in] pCBParam : Callback parameter (unused)
 *
 * @param [in] Event : Event of type #ADI_BLER_EVENT.
 *
 * @param [in] pArg : Callback argument (unused)
 *
 * @note       The application should NOT call other radio functions (adi_ble_radio.c) from this callback that issue
 *             a command to the radio. The application may call radio functions that simply extract data from the
 *             companion module, these are located below #adi_ble_GetEvent in adi_ble_radio.c. Ideally this callback
 *             should just be used for flags and semaphores.
 */
static void ApplicationCallback(void * pCBParam, uint32_t Event, void * pArg)
{
	switch(Event) {
	case GAP_EVENT_CONNECTED:
		DEBUG_MESSAGE("Connected!\r\n");
		gbConnected = true;
		break;

	case GAP_EVENT_DISCONNECTED:
		DEBUG_MESSAGE("Disconnected!\r\n");
		geMode      = ADI_BLE_GAP_MODE_NOTCONNECTABLE;
		gbConnected = false;
		registration_status = false;
		break;

	case DATA_EXCHANGE_RX_EVENT:
		DEBUG_MESSAGE("Data received!\r\n");
		break;

	case DATA_EXCHANGE_TX_COMPLETE:
		DEBUG_MESSAGE("Data sent!\r\n");
		registration_status = true;
		break;

	case GAP_EVENT_MODE_CHANGE:
		DEBUG_MESSAGE("GAP mode changed.\r\n");
		break;

	case GAP_EVENT_CONNECTION_UPDATED:
		DEBUG_MESSAGE("Connection interval updated.\r\n");
		break;

	case BLE_RADIO_ERROR_READING:
		/* If you want to enable this print statement, please be aware that the first
		 * packet sent from the radio on startup will cause this error. It is a known bug
		 * and will not have any adverse effects on the application.
		 *
		 *DEBUG_MESSAGE("Failed to read a packet from the radio.\r\n");
		 *
		 */
		registration_status = false;
		break;

	case BLE_RADIO_ERROR_PARSING:
		DEBUG_MESSAGE("Failed to parse a packet from the radio.\r\n");
		registration_status = false;
		break;

	case BLE_RADIO_ERROR_PROCESSING:
		DEBUG_MESSAGE("Failed to process a packet from the radio.\r\n");
		registration_status = false;
		break;

	default:
		DEBUG_MESSAGE("Unexpected event (%ld) received.\r\n", Event);
		break;
	}
}
