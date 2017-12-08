/**************************************************************************//**
*   @file   AD5421.h
*   @brief  Header file of AD5421 Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification,
* are permitted provided that the following conditions are met:
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
* INCIDENTAL,SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
******************************************************************************/
#ifndef _AD5421_H_
#define _AD5421_H_

/*****************************************************************************/
/******************** Macros and Constants Definitions ***********************/
/*****************************************************************************/
/* COMMAND Bytes */
#define AD5421_CMDWRDAC         1
#define AD5421_CMDWRCTRL        2
#define AD5421_CMDWROFFSET      3
#define AD5421_CMDWRGAIN        4
#define AD5421_CMDRST			7
#define AD5421_CMDMEASVTEMP     8
#define AD5421_CMDRDDAC         129
#define AD5421_CMDRDCTRL		130
#define AD5421_CMDRDOFFSET      131
#define AD5421_CMDRDGAIN        132
#define AD5421_CMDRDFAULT       133

/* AD5421 COMMAND mask */
#define AD5421_CMD(x)			((x & 0xFF) << 16)

/* AD5421 GPIO */
#define AD5421_LDAC_OUT			gpio_direction_output(dev->gpio_ldac,   \
			                GPIO_HIGH)
#define AD5421_LDAC_LOW			gpio_set_value(dev->gpio_ldac,          \
			                GPIO_LOW)
#define AD5421_LDAC_HIGH		gpio_set_value(dev->gpio_ldac,          \
			                GPIO_HIGH)
#define AD5421_FAULT_IN 		gpio_direction_input(dev->gpio_faultin)

/* CONTROL register bits */
#define CTRL_SPI_WATCHDOG		(1 << 12)
#define CTRL_AUTO_FAULT_RDBK    (1 << 11)
#define CTRL_SEL_ADC_INPUT      (1 << 8)
#define CTRL_ONCHIP_ADC         (1 << 7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

typedef struct {
	/* SPI */
	spi_desc		*spi_desc;
	/* GPIO */
	gpio_desc		*gpio_ldac;
	gpio_desc		*gpio_faultin;
} ad5421_dev;

typedef struct {
	/* SPI */
	spi_init_param	spi_init;
	/* GPIO */
	int8_t		gpio_ldac;
	int8_t		gpio_faultin;
} ad5421_init_param;

/*****************************************************************************/
/************************* Functions Declarations ****************************/
/*****************************************************************************/
/* Initialize the communication with the device. */
int   AD5421_Init(ad5421_dev **device,
		  ad5421_init_param init_param);
/* Free the resources allocated by AD5421_Init(). */
int32_t AD5421_remove(ad5421_dev *dev);
/* Set the value of DAC register. */
void  AD5421_SetDac(ad5421_dev *dev,
		    int dacValue);
/* Set the value of OFFSET register. */
void  AD5421_SetOffset(ad5421_dev *dev,
		       int offsetValue);
/* Set the value of GAIN register. */
void  AD5421_SetGain(ad5421_dev *dev,
		     int gainValue);
/* Read the DAC register. */
int   AD5421_GetDac(ad5421_dev *dev);
/* Read OFFSET register. */
int   AD5421_GetOffset(ad5421_dev *dev);
/* Read GAIN register. */
int   AD5421_GetGain(ad5421_dev *dev);
/* Read FAULT register. */
int   AD5421_GetFault(ad5421_dev *dev);
/* Read the temperature from Fault register. */
int   AD5421_GetTemp(ad5421_dev *dev);
/* Read VLoop-COM from Fault register. */
float AD5421_GetVloop(ad5421_dev *dev);
/* Send command via SPI. */
int   AD5421_Set(ad5421_dev *dev,
		 int *iValue);
/* Receive value via SPI. */
int   AD5421_Get(ad5421_dev *dev);
/* Resets the AD5421 device. */
void  AD5421_Reset(ad5421_dev *dev);

#endif /* _AD5421_H_ */
