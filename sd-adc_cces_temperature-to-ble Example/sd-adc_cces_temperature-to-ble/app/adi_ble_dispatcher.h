/*!
 *****************************************************************************
  @file:  adi_ble_dispatcher.h

  @brief: This class transmits samples over BLE

  @details:
 -----------------------------------------------------------------------------
 Copyright (c) 2019, 2020 Analog Devices, Inc.  All rights reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef ADI_BLE_DISPATCHER_H_
#define ADI_BLE_DISPATCHER_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "dispatcher.h"
#include "sampling_engine.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Define the packet type identifiers */
#define REGISTRATION_PACKET_ID	0
#define FIELD_NAME_PACKET_ID	1
#define DATA_PACKET_ID			2

/* Define the packet data type identifiers */
#define PACKET_DATA_TYPE_BYTE	1	// 1 byte
#define PACKET_DATA_TYPE_SHORT	2	// 2 bytes
#define PACKET_DATA_TYPE_INT	3	// 4 bytes
#define PACKET_DATA_TYPE_LONG	4	// 8 bytes
#define PACKET_DATA_TYPE_FLOAT	5	// 4 bytes
#define PACKET_DATA_TYPE_DOUBLE	6	// 8 bytes
#define PACKET_DATA_TYPE_USHORT	7	// 2 bytes

/******************************************************************************/
/********************** Class Declaration *************************************/
/******************************************************************************/

class AdiBleDispatcher : public Dispatcher
{
public:
	AdiBleDispatcher();
	virtual void Idle(const uint16_t nTimeoutMilliSeconds);
	virtual bool isConnected(void);
	virtual DISPATCHER_ERROR_TYPE TransmitTemperature(const float temperatureSample,
			const uint32_t timeStamp, const uint8_t sensorID);
	virtual DISPATCHER_ERROR_TYPE TransmitAxlXYZ(const float axlX, const float axlY,
			const float axlZ,
			const uint32_t timeStamp, const uint8_t sensorID);

	typedef union {
		uint8_t u8Value[4];
		float fValue;
	} floatUnion_t;

#pragma pack(1)
	typedef struct {
		uint8_t pkt_type_sensor_id;
		uint8_t num_fields;
		uint8_t data_type;
		uint8_t sensor_name[17];
	} registration_packet;

#pragma pack(1)
	typedef struct {
		uint8_t pkt_type_sensor_id;
		uint8_t field_id;
		uint8_t field_name[18];
	} field_name_packet;

#pragma pack(1)
	typedef struct {
		uint8_t pkt_type_sensor_id;
		floatUnion_t sensor_data1;
		floatUnion_t sensor_data2;
		floatUnion_t sensor_data3;
		floatUnion_t sensor_data4;
	} data_packet;

private:
	void InitBluetoothLowEnergy(void);
	void SetAdvertisingMode(void);
};

#endif /* ADI_BLE_DISPATCHER_H_ */
