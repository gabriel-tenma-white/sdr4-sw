/***************************************************************************//**
 *   @file   adxl345.c
 *   @brief  Implementation of ADXL345 Driver.
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
#include "adxl345.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t adxl345_get_register_value(adxl345_dev *dev,
				   uint8_t register_address)
{
	uint8_t data_buffer[2] = {0, 0};
	uint8_t register_value = 0;

	if (dev->communication_type == ADXL345_SPI_COMM) {
		data_buffer[0] = ADXL345_SPI_READ | register_address;
		data_buffer[1] = 0;
		spi_write_and_read(&dev->spi_dev,
				   data_buffer,
				   2);
		register_value = data_buffer[1];
	} else {
		i2c_write(&dev->i2c_dev,
			  &register_address, // Transmission data.
			  1,                 // Number of bytes to write.
			  0);                // Stop condition control.
		i2c_read(&dev->i2c_dev,
			 &register_value,    // Received data.
			 1,                  // Number of bytes to read.
			 1);                 // Stop condition control.
	}

	return register_value;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param register_address - Address of the register.
 * @param register_value   - Data value to write.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_register_value(adxl345_dev *dev,
				uint8_t register_address,
				uint8_t register_value)
{
	uint8_t data_buffer[2] = {0, 0};

	if (dev->communication_type == ADXL345_SPI_COMM) {
		data_buffer[0] = ADXL345_SPI_WRITE | register_address;
		data_buffer[1] = register_value;
		spi_write_and_read(&dev->spi_dev,
				   data_buffer,
				   2);
	} else {
		data_buffer[0] = register_address;
		data_buffer[1] = register_value;
		i2c_write(&dev->i2c_dev,
			  data_buffer,        // Received data.
			  2,                  // Number of bytes to read.
			  1);                 // Stop condition control.
	}
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *        part is present.
 *
 * @param device     - The device structure.
 *
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return status    - Result of the initialization procedure.
 *                     Example: -1 - I2C/SPI peripheral was not initialized or
 *                                   ADXL345 part is not present.
 *                               0 - I2C/SPI peripheral is initialized and
 *                                   ADXL345 part is present.
*******************************************************************************/
int32_t adxl345_init(adxl345_dev **device,
		     adxl345_init_param init_param)
{
	adxl345_dev *dev;
	int32_t     status = 0;

	dev = (adxl345_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	dev->communication_type = init_param.communication_type;

	if (dev->communication_type == ADXL345_SPI_COMM) {
		dev->spi_dev.type = init_param.spi_type;
		dev->spi_dev.id = init_param.spi_id;
		dev->spi_dev.max_speed_hz = init_param.spi_max_speed_hz;
		dev->spi_dev.mode = init_param.spi_mode;
		dev->spi_dev.chip_select = init_param.spi_chip_select;
		status = spi_init(&dev->spi_dev);
	} else {
		dev->i2c_dev.type = init_param.i2c_type;
		dev->i2c_dev.id = init_param.i2c_id;
		dev->i2c_dev.max_speed_hz = init_param.i2c_max_speed_hz;
		dev->i2c_dev.slave_address = init_param.i2c_slave_address;
		status = i2c_init(&dev->i2c_dev);
	}
	if (adxl345_get_register_value(dev, ADXL345_DEVID) != ADXL345_ID)
		status = -1;

	dev->selected_range = 2; // Measurement Range: +/- 2g (reset default).
	dev->full_resolution_set = 0;

	*device = dev;

	return status;
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwr_mode - Power mode.
 *                   Example: 0x0 - standby mode.
 *                            0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_power_mode(adxl345_dev *dev,
			    uint8_t pwr_mode)
{
	uint8_t old_power_ctl = 0;
	uint8_t new_power_ctl = 0;

	old_power_ctl = adxl345_get_register_value(dev,
			ADXL345_POWER_CTL);
	new_power_ctl = old_power_ctl & ~ADXL345_PCTL_MEASURE;
	new_power_ctl = new_power_ctl | (pwr_mode * ADXL345_PCTL_MEASURE);
	adxl345_set_register_value(dev,
				   ADXL345_POWER_CTL,
				   new_power_ctl);
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_xyz(adxl345_dev *dev,
		     int16_t* x,
		     int16_t* y,
		     int16_t* z)
{
	uint8_t first_reg_address = ADXL345_DATAX0;
	uint8_t read_buffer[7]    = {0, 0, 0, 0, 0, 0, 0};

	if (dev->communication_type == ADXL345_SPI_COMM) {
		read_buffer[0] = ADXL345_SPI_READ |
				 ADXL345_SPI_MB |
				 first_reg_address;
		spi_write_and_read(&dev->spi_dev,
				   read_buffer,
				   7);
		/* x = ((ADXL345_DATAX1) << 8) + ADXL345_DATAX0 */
		*x = ((int16_t)read_buffer[2] << 8) + read_buffer[1];
		/* y = ((ADXL345_DATAY1) << 8) + ADXL345_DATAY0 */
		*y = ((int16_t)read_buffer[4] << 8) + read_buffer[3];
		/* z = ((ADXL345_DATAZ1) << 8) + ADXL345_DATAZ0 */
		*z = ((int16_t)read_buffer[6] << 8) + read_buffer[5];
	} else {
		i2c_write(&dev->i2c_dev,
			  &first_reg_address, // Transmission data.
			  1,                  // Number of bytes to write.
			  0);                 // Stop condition control.
		i2c_read(&dev->i2c_dev,
			 read_buffer,         // Received data.
			 6,                   // Number of bytes to read.
			 1);                  // Stop condition control.
		/* x = ((ADXL345_DATAX1) << 8) + ADXL345_DATAX0 */
		*x = ((int16_t)read_buffer[1] << 8) + read_buffer[0];
		/* y = ((ADXL345_DATAY1) << 8) + ADXL345_DATAY0 */
		*y = ((int16_t)read_buffer[3] << 8) + read_buffer[2];
		/* z = ((ADXL345_DATAZ1) << 8) + ADXL345_DATAZ0 */
		*z = ((int16_t)read_buffer[5] << 8) + read_buffer[4];
	}
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_g_xyz(adxl345_dev *dev,
		       float* x,
		       float* y,
		       float* z)
{
	int16_t x_data = 0;  // X-axis's output data.
	int16_t y_data = 0;  // Y-axis's output data.
	int16_t z_data = 0;  // Z-axis's output data.

	adxl345_get_xyz(dev, &x_data, &y_data, &z_data);
	*x = (float)(dev->full_resolution_set ? (x_data * ADXL345_SCALE_FACTOR) :
		     (x_data * ADXL345_SCALE_FACTOR * (dev->selected_range >> 1)));
	*y = (float)(dev->full_resolution_set ? (y_data * ADXL345_SCALE_FACTOR) :
		     (y_data * ADXL345_SCALE_FACTOR * (dev->selected_range >> 1)));
	*z = (float)(dev->full_resolution_set ? (z_data * ADXL345_SCALE_FACTOR) :
		     (z_data * ADXL345_SCALE_FACTOR * (dev->selected_range >> 1)));
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tap_type   - Tap type (none, single, double).
 *                     Example: 0x0 - disables tap detection.
 *				ADXL345_SINGLE_TAP - enables single tap
 *                                                   detection.
 *				ADXL345_DOUBLE_TAP - enables double tap
 *                                                   detection.
 * @param tap_axes   - Axes which participate in tap detection.
 *                     Example: 0x0 - disables axes participation.
 *				ADXL345_TAP_X_EN - enables x-axis participation.
 *				ADXL345_TAP_Y_EN - enables y-axis participation.
 *				ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tap_dur    - Tap duration. The scale factor is 625us is/LSB.
 * @param tap_latent - Tap latency. The scale factor is 1.25 ms/LSB.
 * @param tap_window - Tap window. The scale factor is 1.25 ms/LSB.
 * @param tap_thresh - Tap threshold. The scale factor is 62.5 mg/LSB.
 * @param tap_int    - Interrupts pin.
 *                     Example: 0x0 - interrupts on INT1 pin.
 *				ADXL345_SINGLE_TAP - single tap interrupts on
 *						     INT2 pin.
 *				ADXL345_DOUBLE_TAP - double tap interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_tap_detection(adxl345_dev *dev,
			       uint8_t tap_type,
			       uint8_t tap_axes,
			       uint8_t tap_dur,
			       uint8_t tap_latent,
			       uint8_t tap_window,
			       uint8_t tap_thresh,
			       uint8_t tap_int)
{
	uint8_t old_tap_axes   = 0;
	uint8_t new_tap_axes   = 0;
	uint8_t old_int_map    = 0;
	uint8_t new_int_map    = 0;
	uint8_t old_int_enable = 0;
	uint8_t new_int_enable = 0;

	old_tap_axes = adxl345_get_register_value(dev,
			ADXL345_TAP_AXES);
	new_tap_axes = old_tap_axes & ~(ADXL345_TAP_X_EN |
					ADXL345_TAP_Y_EN |
					ADXL345_TAP_Z_EN);
	new_tap_axes = new_tap_axes | tap_axes;
	adxl345_set_register_value(dev,
				   ADXL345_TAP_AXES,
				   new_tap_axes);
	adxl345_set_register_value(dev,
				   ADXL345_DUR,
				   tap_dur);
	adxl345_set_register_value(dev,
				   ADXL345_LATENT,
				   tap_latent);
	adxl345_set_register_value(dev,
				   ADXL345_WINDOW,
				   tap_window);
	adxl345_set_register_value(dev,
				   ADXL345_THRESH_TAP,
				   tap_thresh);
	old_int_map = adxl345_get_register_value(dev,
			ADXL345_INT_MAP);
	new_int_map = old_int_map &
		      ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
	new_int_map = new_int_map | tap_int;
	adxl345_set_register_value(dev,
				   ADXL345_INT_MAP,
				   new_int_map);
	old_int_enable = adxl345_get_register_value(dev,
			 ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable &
			 ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
	new_int_enable = new_int_enable | tap_type;
	adxl345_set_register_value(dev,
				   ADXL345_INT_ENABLE,
				   new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff  - Enables/disables the activity detection.
 *                    Example: 0x0 - disables the activity detection.
 *                             0x1 - enables the activity detection.
 * @param actAxes   - Axes which participate in detecting activity.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_ACT_X_EN - enables x-axis participation.
 *                             ADXL345_ACT_Y_EN - enables y-axis participation.
 *                             ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc   - Selects dc-coupled or ac-coupled operation.
 *                    Example: 0x0 - dc-coupled operation.
 *                             ADXL345_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity. The scale factor
                      is 62.5 mg/LSB.
 * @patam actInt    - Interrupts pin.
 *                    Example: 0x0 - activity interrupts on INT1 pin.
 *                             ADXL345_ACTIVITY - activity interrupts on INT2
 *                                                pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_activity_detection(adxl345_dev *dev,
				    uint8_t actOnOff,
				    uint8_t actAxes,
				    uint8_t actAcDc,
				    uint8_t actThresh,
				    uint8_t actInt)
{
	uint8_t old_act_inact_ctl = 0;
	uint8_t new_act_inact_ctl = 0;
	uint8_t old_int_map       = 0;
	uint8_t new_int_map       = 0;
	uint8_t old_int_enable    = 0;
	uint8_t new_int_enable    = 0;

	old_act_inact_ctl = adxl345_get_register_value(dev,
			    ADXL345_INT_ENABLE);
	new_act_inact_ctl = old_act_inact_ctl & ~(ADXL345_ACT_ACDC |
			    ADXL345_ACT_X_EN |
			    ADXL345_ACT_Y_EN |
			    ADXL345_ACT_Z_EN);
	new_act_inact_ctl = new_act_inact_ctl | (actAcDc | actAxes);
	adxl345_set_register_value(dev,
				   ADXL345_ACT_INACT_CTL,
				   new_act_inact_ctl);
	adxl345_set_register_value(dev,
				   ADXL345_THRESH_ACT,
				   actThresh);
	old_int_map = adxl345_get_register_value(dev,
			ADXL345_INT_MAP);
	new_int_map = old_int_map & ~(ADXL345_ACTIVITY);
	new_int_map = new_int_map | actInt;
	adxl345_set_register_value(dev,
				   ADXL345_INT_MAP,
				   new_int_map);
	old_int_enable = adxl345_get_register_value(dev,
			 ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable & ~(ADXL345_ACTIVITY);
	new_int_enable = new_int_enable | (ADXL345_ACTIVITY * actOnOff);
	adxl345_set_register_value(dev,
				   ADXL345_INT_ENABLE,
				   new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inact_on_off - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inact_axes   - Axes which participate in detecting inactivity.
 *                       Example: 0x0 - disables axes participation.
 *                                ADXL345_INACT_X_EN - enables x-axis.
 *                                ADXL345_INACT_Y_EN - enables y-axis.
 *                                ADXL345_INACT_Z_EN - enables z-axis.
 * @param inact_ac_dc  - Selects dc-coupled or ac-coupled operation.
 *                       Example: 0x0 - dc-coupled operation.
 *                                ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inact_thresh - Threshold value for detecting inactivity. The scale
                         factor is 62.5 mg/LSB.
 * @param inact_time   - Inactivity time. The scale factor is 1 sec/LSB.
 * @patam inact_int    - Interrupts pin.
 *		         Example: 0x0 - inactivity interrupts on INT1 pin.
 *				  ADXL345_INACTIVITY - inactivity interrupts on
 *						       INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_inactivity_detection(adxl345_dev *dev,
				      uint8_t inact_on_off,
				      uint8_t inact_axes,
				      uint8_t inact_ac_dc,
				      uint8_t inact_thresh,
				      uint8_t inact_time,
				      uint8_t inact_int)
{
	uint8_t old_act_inact_ctl = 0;
	uint8_t new_act_inact_ctl = 0;
	uint8_t old_int_map       = 0;
	uint8_t new_int_map       = 0;
	uint8_t old_int_enable    = 0;
	uint8_t new_int_enable    = 0;

	old_act_inact_ctl = adxl345_get_register_value(dev,
			    ADXL345_INT_ENABLE);
	new_act_inact_ctl = old_act_inact_ctl & ~(ADXL345_INACT_ACDC |
			    ADXL345_INACT_X_EN |
			    ADXL345_INACT_Y_EN |
			    ADXL345_INACT_Z_EN);
	new_act_inact_ctl = new_act_inact_ctl | (inact_ac_dc | inact_axes);
	adxl345_set_register_value(dev,
				   ADXL345_ACT_INACT_CTL,
				   new_act_inact_ctl);
	adxl345_set_register_value(dev,
				   ADXL345_THRESH_INACT,
				   inact_thresh);
	adxl345_set_register_value(dev,
				   ADXL345_TIME_INACT,
				   inact_time);
	old_int_map = adxl345_get_register_value(dev,
			ADXL345_INT_MAP);
	new_int_map = old_int_map & ~(ADXL345_INACTIVITY);
	new_int_map = new_int_map | inact_int;
	adxl345_set_register_value(dev,
				   ADXL345_INT_MAP,
				   new_int_map);
	old_int_enable = adxl345_get_register_value(dev,
			 ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable & ~(ADXL345_INACTIVITY);
	new_int_enable = new_int_enable | (ADXL345_INACTIVITY * inact_on_off);
	adxl345_set_register_value(dev,
				   ADXL345_INT_ENABLE,
				   new_int_enable);
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ff_on_off - Enables/disables the free-fall detection.
 *                    Example: 0x0 - disables the free-fall detection.
 *                             0x1 - enables the free-fall detection.
 * @param ff_thresh - Threshold value for free-fall detection. The scale factor
 *                    is 62.5 mg/LSB.
 * @param ff_time   - Time value for free-fall detection. The scale factor is
 *                    5 ms/LSB.
 * @param ff_int    - Interrupts pin.
 *		      Example: 0x0 - free-fall interrupts on INT1 pin.
 *			       ADXL345_FREE_FALL - free-fall interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_free_fall_detection(adxl345_dev *dev,
				     uint8_t ff_on_off,
				     uint8_t ff_thresh,
				     uint8_t ff_time,
				     uint8_t ff_int)
{
	uint8_t old_int_map    = 0;
	uint8_t new_int_map    = 0;
	uint8_t old_int_enable = 0;
	uint8_t new_int_enable = 0;

	adxl345_set_register_value(dev,
				   ADXL345_THRESH_FF,
				   ff_thresh);
	adxl345_set_register_value(dev,
				   ADXL345_TIME_FF,
				   ff_time);
	old_int_map = adxl345_get_register_value(dev,
			ADXL345_INT_MAP);
	new_int_map = old_int_map & ~(ADXL345_FREE_FALL);
	new_int_map = new_int_map | ff_int;
	adxl345_set_register_value(dev,
				   ADXL345_INT_MAP,
				   new_int_map);
	old_int_enable = adxl345_get_register_value(dev,
			 ADXL345_INT_ENABLE);
	new_int_enable = old_int_enable & ~ADXL345_FREE_FALL;
	new_int_enable = new_int_enable | (ADXL345_FREE_FALL * ff_on_off);
	adxl345_set_register_value(dev,
				   ADXL345_INT_ENABLE,
				   new_int_enable);
}

/***************************************************************************//**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param x_offset - X-axis's offset.
 * @param y_offset - Y-axis's offset.
 * @param z_offset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_offset(adxl345_dev *dev,
			uint8_t x_offset,
			uint8_t y_offset,
			uint8_t z_offset)
{
	adxl345_set_register_value(dev,
				   ADXL345_OFSX,
				   x_offset);
	adxl345_set_register_value(dev,
				   ADXL345_OFSY,
				   y_offset);
	adxl345_set_register_value(dev,
				   ADXL345_OFSZ,
				   z_offset);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param g_range  - Range option.
 *                   Example: ADXL345_RANGE_PM_2G  - +-2 g
 *                            ADXL345_RANGE_PM_4G  - +-4 g
 *                            ADXL345_RANGE_PM_8G  - +-8 g
 *                            ADXL345_RANGE_PM_16G - +-16 g
 * @param full_res - Full resolution option.
 *                   Example: 0x0 - Disables full resolution.
 *                            ADXL345_FULL_RES - Enables full resolution.
 *
 * @return None.
*******************************************************************************/
void ADXL345_set_range_resolution(adxl345_dev *dev,
				  uint8_t g_range,
				  uint8_t full_res)
{
	uint8_t old_data_format = 0;
	uint8_t new_data_format = 0;

	old_data_format = adxl345_get_register_value(dev,
			  ADXL345_DATA_FORMAT);
	new_data_format = old_data_format &
			  ~(ADXL345_RANGE(0x3) | ADXL345_FULL_RES);
	new_data_format =  new_data_format | ADXL345_RANGE(g_range) | full_res;
	adxl345_set_register_value(dev,
				   ADXL345_DATA_FORMAT,
				   new_data_format);
	dev->selected_range = (1 << (g_range + 1));
	dev->full_resolution_set = full_res ? 1 : 0;
}
