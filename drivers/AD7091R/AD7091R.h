/***************************************************************************//**
 *   @file   AD7091R.h
 *   @brief  Header file of AD7091R Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
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
*******************************************************************************/

#ifndef __AD7091R_H__
#define __AD7091R_H__

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

struct ad7091r_dev {
	/* SPI */
	spi_desc	*spi_desc;
};

struct ad7091r_init_param {
	/* SPI */
	spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Initializes the SPI communication peripheral. */
int8_t ad7091r_init(struct ad7091r_dev **device,
		    struct ad7091r_init_param init_param);

/*! Free the resources allocated by ad7091r_init(). */
int32_t ad5686_remove(struct ad7091r_dev *dev);

/*! Initiate a software reset of the device. */
void ad7091r_software_reset(struct ad7091r_dev *dev);

/*! Initiates one conversion and reads back the result. */
uint16_t ad7091r_read_sample(struct ad7091r_dev *dev);

/*! Puts the device in power-down mode. */
void ad7091r_power_down(struct ad7091r_dev *dev);

/*! Powers up the device. */
void ad7091r_power_up(struct ad7091r_dev *dev);

/*! Converts a 12-bit raw sample to volts. */
float ad7091r_convert_to_volts(int16_t raw_sample, float v_ref);

#endif /* __AD7091R_H__ */
