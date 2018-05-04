/***************************************************************************//**
 *   @file   adt7420.c
 *   @brief  Implementation of ADT7420 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "platform_drivers.h"
#include "adt7420.h"

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t adt7420_get_register_value(adt7420_dev *dev,
				   uint8_t register_address)
{
	uint8_t register_value = 0;

	i2c_write(&dev->i2c_dev,
		  &register_address,
		  1,
		  0);
	i2c_read(&dev->i2c_dev,
		 &register_value,
		 1,
		 1);

	return register_value;
}

/***************************************************************************//**
 * @brief Sets the value of a register.
 *
 * @param register_address - Address of the register.
 * @param register_value   - Value of the register.
 *
 * @return None.
*******************************************************************************/
void adt7420_set_register_value(adt7420_dev *dev,
				uint8_t register_address,
				uint8_t register_value)
{
	uint8_t data_buffer[2] = {0, 0};

	data_buffer[0] = register_address;
	data_buffer[1] = register_value;
	i2c_write(&dev->i2c_dev,
		  data_buffer,
		  2,
		  1);
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the device is
 *        present.
 *
 * @return status - The result of the initialization procedure.
 *                  Example: -1 - I2C peripheral was not initialized or the
 *                                device is not present.
 *                            0 - I2C peripheral was initialized and the
 *                                device is present.
*******************************************************************************/
int32_t adt7420_init(adt7420_dev **device,
		     adt7420_init_param init_param)
{
	adt7420_dev *dev;
	int32_t status;
	uint8_t test   = 0;

	dev = (adt7420_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	/* I2C */
	dev->i2c_dev.type = init_param.i2c_type;
	dev->i2c_dev.id = init_param.i2c_id;
	dev->i2c_dev.max_speed_hz = init_param.i2c_max_speed_hz;
	dev->i2c_dev.slave_address = init_param.i2c_slave_address;
	status = i2c_init(&dev->i2c_dev);

	/* Device Settings */
	dev->resolution_setting = init_param.resolution_setting;

	test   = adt7420_get_register_value(dev, ADT7420_REG_ID);
	if(test != ADT7420_DEFAULT_ID)
		status = -1;

	*device = dev;

	return status;
}

/***************************************************************************//**
 * @brief Resets the ADT7420.
 *        The ADT7420 does not respond to I2C bus commands while the default
 *        values upload (approximately 200 us).
 *
 * @return None.
*******************************************************************************/
void adt7420_reset(adt7420_dev *dev)
{
	uint8_t register_address = ADT7420_REG_RESET;

	i2c_write(&dev->i2c_dev,
		  &register_address,
		  1,
		  1);
	dev->resolution_setting = 0;
}

/***************************************************************************//**
 * @brief Sets the operational mode for ADT7420.
 *
 * @param mode - Operation mode.
 *               Example: ADT7420_OP_MODE_CONT_CONV - continuous conversion;
 *                        ADT7420_OP_MODE_ONE_SHOT  - one shot;
 *                        ADT7420_OP_MODE_1_SPS     - 1 SPS mode;
 *                        ADT7420_OP_MODE_SHUTDOWN  - shutdown.
 *
 * @return None.
*******************************************************************************/
void adt7420_set_operation_mode(adt7420_dev *dev,
				uint8_t mode)
{
	uint8_t register_value = 0;

	register_value  = adt7420_get_register_value(dev, ADT7420_REG_CONFIG);
	register_value &= ~ADT7420_CONFIG_OP_MODE(ADT7420_OP_MODE_SHUTDOWN);
	register_value |= ADT7420_CONFIG_OP_MODE(mode);
	adt7420_set_register_value(dev, ADT7420_REG_CONFIG, register_value);
}

/***************************************************************************//**
 * @brief Sets the resolution for ADT7420.
 *
 * @param resolution - Resolution.
 *                     Example: 0 - 13-bit resolution;
 *                              1 - 16-bit resolution.
 *
 * @return None.
*******************************************************************************/
void adt7420_set_resolution(adt7420_dev *dev,
			    uint8_t resolution)
{
	uint8_t register_value = 0;

	register_value  = adt7420_get_register_value(dev, ADT7420_REG_CONFIG);
	register_value &= ~ADT7420_CONFIG_RESOLUTION;
	register_value |= (resolution * ADT7420_CONFIG_RESOLUTION);
	adt7420_set_register_value(dev, ADT7420_REG_CONFIG, register_value);
	dev->resolution_setting = resolution;
}

/***************************************************************************//**
 * @brief Reads the temperature data and converts it to Celsius degrees.
 *
 * @return temperature - Temperature in degrees Celsius.
*******************************************************************************/
float adt7420_get_temperature(adt7420_dev *dev)
{
	uint8_t  msb_temp = 0;
	uint8_t  lsb_temp = 0;
	uint16_t temp     = 0;
	float    temp_c   = 0;

	msb_temp = adt7420_get_register_value(dev, ADT7420_REG_TEMP_MSB);
	lsb_temp = adt7420_get_register_value(dev, ADT7420_REG_TEMP_LSB);
	temp    = ((uint16_t)msb_temp << 8) + lsb_temp;
	if(dev->resolution_setting) {
		if(temp & 0x8000)
			/*! Negative temperature */
			temp_c = (float)((int32_t)temp - 65536) / 128;
		else
			/*! Positive temperature */
			temp_c = (float)temp / 128;
	} else {
		temp >>= 3;
		if(temp & 0x1000)
			/*! Negative temperature */
			temp_c = (float)((int32_t)temp - 8192) / 16;
		else
			/*! Positive temperature */
			temp_c = (float)temp / 16;
	}

	return temp_c;
}
