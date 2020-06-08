/***************************************************************************//**
 *   @file   platform_drivers.c
 *   @brief  Implementation of Platform Drivers for ADuCM3029
 *   @author
********************************************************************************
 * Copyright (c) 2017-2018, 2020 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

/* ADuCM3029 driver includes */
#include <common.h>
#include <drivers/general/adi_drivers_general.h>

#include <drivers/spi/adi_spi.h>
#include <drivers/dma/adi_dma.h>

#include "platform_drivers.h"

/******************************************************************************/
/************************ Variable Declarations *******************************/
/******************************************************************************/

/* Transmit data buffer */
ADI_ALIGNED_PRAGMA(2)
static uint8_t overtx[BUFFERSIZE] ADI_ALIGNED_ATTRIBUTE(2);

/* Receive data buffer a */
ADI_ALIGNED_PRAGMA(2)
static uint8_t overrx[BUFFERSIZE] ADI_ALIGNED_ATTRIBUTE(2);

// the platform drivers only allow one device to call spi_init()
static     ADI_SPI_HANDLE hDevice = NULL;

/* the "usable" buffers within the over-allocated buffers housing the guard data */

ADI_ALIGNED_PRAGMA(2)
uint8_t spidevicemem[ADI_SPI_MEMORY_SIZE] ADI_ALIGNED_ATTRIBUTE(2);



/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/


/**
 * @brief Initialize the I2C communication peripheral.
 * @param desc - The I2C descriptor.
 * @param init_param - The structure that contains the I2C parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_init(struct i2c_desc **desc,
		 const struct i2c_init_param *param)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (param->type) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

/**
 * @brief Free the resources allocated by i2c_init().
 * @param desc - The I2C descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_remove(struct i2c_desc *desc)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

/**
 * @brief Write data to a slave device.
 * @param desc - The I2C descriptor.
 * @param data - Buffer that stores the transmission data.
 * @param bytes_number - Number of bytes to write.
 * @param stop_bit - Stop condition control.
 *                   Example: 0 - A stop condition will not be generated;
 *                            1 - A stop condition will be generated.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_write(struct i2c_desc *desc,
		  uint8_t *data,
		  uint8_t bytes_number,
		  uint8_t stop_bit)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (data) {
		// Unused variable - fix compiler warning
	}

	if (bytes_number) {
		// Unused variable - fix compiler warning
	}

	if (stop_bit) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

/**
 * @brief Read data from a slave device.
 * @param desc - The I2C descriptor.
 * @param data - Buffer that will store the received data.
 * @param bytes_number - Number of bytes to read.
 * @param stop_bit - Stop condition control.
 *                   Example: 0 - A stop condition will not be generated;
 *                            1 - A stop condition will be generated.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_read(struct i2c_desc *desc,
		 uint8_t *data,
		 uint8_t bytes_number,
		 uint8_t stop_bit)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (data) {
		// Unused variable - fix compiler warning
	}

	if (bytes_number) {
		// Unused variable - fix compiler warning
	}

	if (stop_bit) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

/**
 * @brief Initialize the SPI communication peripheral.
 * @param desc - The SPI descriptor.
 * @param init_param - The structure that contains the SPI parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
	ADI_SPI_RESULT eResult = ADI_SPI_FAILURE;

	if (desc && param) {
		// Create the spi description object for the device
		spi_desc * new_desc = (spi_desc *)malloc(sizeof(spi_desc));
		if (new_desc == NULL) {
			return FAILURE;
		}

		new_desc->chip_select = param->chip_select;
		new_desc->mode = param->mode;
		new_desc->max_speed_hz = param->max_speed_hz;
		new_desc->id = param->id;
		new_desc->type = param->type;

		// SPI_DEVICE_NUM is fixed for now, not possible choose a different SPI peripheral
		eResult = adi_spi_Open(SPI_DEVICE_NUM, spidevicemem, ADI_SPI_MEMORY_SIZE,
				       &hDevice);
		DEBUG_RESULT("Failed to init SPI driver", eResult, ADI_SPI_SUCCESS);

		/* throttle bitrate to something the controller can reach */
		eResult = adi_spi_SetBitrate(hDevice, param->max_speed_hz);
		DEBUG_RESULT("Failed to set Bitrate", eResult, ADI_SPI_SUCCESS);

		bool clockPolarity = ( param->mode == SPI_MODE_2 || param->mode == SPI_MODE_3);
		eResult = adi_spi_SetClockPolarity(hDevice, clockPolarity);
		DEBUG_RESULT("Failed to set SPI Clock Polarity", eResult, ADI_SPI_SUCCESS);

		bool clockPhase = ( param->mode == SPI_MODE_1 || param->mode == SPI_MODE_3);
		eResult = adi_spi_SetClockPhase(hDevice, clockPhase);
		DEBUG_RESULT("Failed to set SPI Clock Phase", eResult, ADI_SPI_SUCCESS);

		/* Set IRQMODE. In this case we are setting it to the default value  */
		/* This code sequence is just calling out the fact that this API would be required  */
		/* for short bursts (less than the size of the FIFO) in PIO (interrupt) mode        */
		eResult = adi_spi_SetIrqmode(hDevice, 0u);
		DEBUG_RESULT("Failed to set Irqmode", eResult, ADI_SPI_SUCCESS);

		/* set the chip select */
		ADI_SPI_CHIP_SELECT chipSelect = (param->chip_select == 0) ? ADI_SPI_CS_NONE :
						 (1 << param->chip_select);
		eResult = adi_spi_SetChipSelect(hDevice, chipSelect );
		DEBUG_RESULT("Failed to set the chip select", eResult, ADI_SPI_SUCCESS);

		/* Set the CS to be continuous during a transaction */
		eResult = adi_spi_SetContinuousMode(hDevice, true);
		DEBUG_RESULT("Failed to set the Continuous CSB Mode", eResult, ADI_SPI_SUCCESS);

#ifdef ENABLE_INTERNAL_SPI_LOOPBACK
		/* set internal loopback mode */
		eResult = adi_spi_SetLoopback(hDevice, true);
		DEBUG_RESULT("Failed to set internal loopback mode",eResult,ADI_SPI_SUCCESS);
#endif

		*desc = new_desc;
	}

	return (eResult == ADI_SPI_SUCCESS ? SUCCESS : FAILURE);
}

/**
 * @brief Free the resources allocated by spi_init().
 * @param desc - The SPI descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_remove(struct spi_desc *desc)
{
	if (desc) {
		// Free the SPI descriptor object
		free(desc);
		return SUCCESS;
	}

	return SUCCESS;
}

/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint8_t bytes_number)
{
	ADI_SPI_RESULT result;
	ADI_SPI_TRANSCEIVER transceive;

	if (desc) {
		// Copy the data to be transmitted to the overtx file local array
		memcpy(overtx, data, bytes_number);

		/* link transceive data size to the remaining count */
		transceive.TransmitterBytes = bytes_number;
		/* link transceive data size to the remaining count */
		transceive.ReceiverBytes = bytes_number;
		/* initialize data attributes */
		transceive.pTransmitter = overtx;
		transceive.pReceiver = overrx;
		/* auto increment both buffers */
		transceive.nTxIncrement = 1;
		transceive.nRxIncrement = 1;
		transceive.bDMA = true;
		transceive.bRD_CTL = false;

		if (ADI_SPI_SUCCESS != (result = adi_spi_MasterReadWrite(hDevice,
						 &transceive))) {
			return FAILURE;
		}

		/* Copy the overrx buffer to the supplied data buffer to return to caller*/
		memcpy(data, overrx, bytes_number);

		return SUCCESS;
	}

	return FAILURE;
}

/**
 * @brief Obtain the GPIO descriptor.
 * @param desc - The GPIO descriptor.
 * @param gpio_number - The number of the GPIO.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get(struct gpio_desc **desc,
		 uint8_t gpio_number)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (gpio_number) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Free the resources allocated by gpio_get().
 * @param desc - The SPI descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_remove(struct gpio_desc *desc)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

/**
 * @brief Enable the input direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_direction_input(struct gpio_desc *desc)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Enable the output direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_direction_output(struct gpio_desc *desc,
			      uint8_t value)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (value) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Get the direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param direction - The direction.
 *                    Example: GPIO_OUT
 *                             GPIO_IN
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get_direction(struct gpio_desc *desc,
			   uint8_t *direction)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (direction) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Set the value of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_set_value(struct gpio_desc *desc,
		       uint8_t value)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (value) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Get the value of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get_value(struct gpio_desc *desc,
		       uint8_t *value)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	if (value) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/**
 * @brief Generate milliseconds delay.
 * @param msecs - Delay in milliseconds.
 * @return None.
 */
void mdelay(uint32_t msecs)
{
	if (msecs) {
		// Unused variable - fix compiler warning
	}
}
