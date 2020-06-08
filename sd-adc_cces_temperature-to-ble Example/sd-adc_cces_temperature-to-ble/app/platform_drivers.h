/***************************************************************************//**
 *   @file   platform_drivers.h
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

#ifndef PLATFORM_DRIVERS_H_
#define PLATFORM_DRIVERS_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
/* ADuCM3029 driver includes */
#include <drivers/spi/adi_spi.h>

/******************************************************************************/
/***************************** #defines ***************************************/
/******************************************************************************/

/* Choose a SPI device */
#define SPI_DEVICE_NUM    0U

/** define size of data buffers, DMA max size is 255 */
#define BUFFERSIZE 251

/*
 *  In RD_CTL mode the master will first transmit between 1 and 16 bytes
 *  This mode is not currently used by the platform drivers.
 */
//#define SPI_RD_CTL_NBYTES_TX   3
//#define SPI_RD_CTL_NBYTES_RX   5

/* enable internal loopback for testing without physical loopback jumper */
//#define ENABLE_INTERNAL_SPI_LOOPBACK

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* NOTE added #ifndef protection to avoid redefinition conflict with
 * same named items in "system_ADuCM3029.h" which must be handled by the
 * preprocessor before this file is included*/
#ifndef SUCCESS
#define SUCCESS		0
#endif
#ifndef FAILURE
#define FAILURE		-1
#endif

#define	SPI_CPHA	0x01
#define	SPI_CPOL	0x02

#define GPIO_OUT	0x01
#define GPIO_IN		0x00

#define GPIO_HIGH	0x01
#define GPIO_LOW	0x00

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

typedef enum i2c_type {
	GENERIC_I2C
} i2c_type;

typedef struct i2c_init_param {
	enum i2c_type	type;
	uint32_t	id;
	uint32_t	max_speed_hz;
	uint8_t		slave_address;
} i2c_init_param;

typedef struct i2c_desc {
	enum i2c_type	type;
	uint32_t	id;
	uint32_t	max_speed_hz;
	uint8_t		slave_address;
} i2c_desc;

typedef enum spi_type {
	GENERIC_SPI
} spi_type;

typedef enum spi_mode {
	SPI_MODE_0 = (0 | 0),
	SPI_MODE_1 = (0 | SPI_CPHA),
	SPI_MODE_2 = (SPI_CPOL | 0),
	SPI_MODE_3 = (SPI_CPOL | SPI_CPHA)
} spi_mode;

typedef struct spi_init_param {
	enum spi_type	type;
	uint32_t	id;
	uint32_t	max_speed_hz;
	enum spi_mode	mode;
	uint8_t		chip_select;
} spi_init_param;

typedef struct spi_desc {
	enum spi_type	type;
	uint32_t	id;
	uint32_t	max_speed_hz;
	enum spi_mode	mode;
	uint8_t		chip_select;
} spi_desc;

typedef enum gpio_type {
	GENERIC_GPIO
} gpio_type;

typedef struct gpio_desc {
	enum gpio_type	type;
	uint32_t	id;
	uint8_t		number;
} gpio_desc;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/


/* Initialize the I2C communication peripheral. */
int32_t i2c_init(struct i2c_desc **desc,
		 const struct i2c_init_param *param);

/* Free the resources allocated by i2c_init(). */
int32_t i2c_remove(struct i2c_desc *desc);

/* Write data to a slave device. */
int32_t i2c_write(struct i2c_desc *desc,
		  uint8_t *data,
		  uint8_t bytes_number,
		  uint8_t stop_bit);

/* Read data from a slave device. */
int32_t i2c_read(struct i2c_desc *desc,
		 uint8_t *data,
		 uint8_t bytes_number,
		 uint8_t stop_bit);

/* Initialize the SPI communication peripheral. */
int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param);

/* Free the resources allocated by spi_init() */
int32_t spi_remove(struct spi_desc *desc);

/* Write and read data to/from SPI. */
int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint8_t bytes_number);

/* Obtain the GPIO descriptor. */
int32_t gpio_get(struct gpio_desc **desc,
		 uint8_t gpio_number);

/* Free the resources allocated by gpio_get() */
int32_t gpio_remove(struct gpio_desc *desc);

/* Enable the input direction of the specified GPIO. */
int32_t gpio_direction_input(struct gpio_desc *desc);

/* Enable the output direction of the specified GPIO. */
int32_t gpio_direction_output(struct gpio_desc *desc,
			      uint8_t value);

/* Get the direction of the specified GPIO. */
int32_t gpio_get_direction(struct gpio_desc *desc,
			   uint8_t *direction);

/* Set the value of the specified GPIO. */
int32_t gpio_set_value(struct gpio_desc *desc,
		       uint8_t value);

/* Get the value of the specified GPIO. */
int32_t gpio_get_value(struct gpio_desc *desc,
		       uint8_t *value);

/* Generate milliseconds delay. */
void mdelay(uint32_t msecs);


#ifdef __cplusplus
}
#endif


#endif // PLATFORM_DRIVERS_H_
