/***************************************************************************//**
*   @file   AD5449.h
*   @brief  Header file of AD5449 Driver. This driver supporting the following
*              devices: AD5415, AD5443, AD5432, AD5426, AD5429, AD5439, AD5449
*
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
********************************************************************************
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
*******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
typedef enum {
    false,
    true
} bool_t;

/* Supported devices */
typedef enum {
    ID_AD5415,
    ID_AD5426,
    ID_AD5429,
    ID_AD5432,
    ID_AD5439,
    ID_AD5443,
    ID_AD5449,
} AD5449_type_t;

typedef struct {
    unsigned char num_channels;
    unsigned char resolution;
    bool_t has_ctrl;
} ad5449_chip_info;

typedef struct {
	/* SPI */
	spi_desc		*spi_desc;
	/* GPIO */
	gpio_desc		*gpio_ldac;
	gpio_desc		*gpio_clr;
	/* Device Settings */
	AD5449_type_t		act_device;
	unsigned short		controlReg;
} ad5449_dev;

typedef struct {
	/* SPI */
	spi_init_param	spi_init;
	/* GPIO */
	int8_t		gpio_ldac;
	int8_t		gpio_clr;
	/* Device Settings */
	AD5449_type_t	act_device;
} ad5449_init_param;

/* Control Bits */
#define AD5449_CTRL_NOP             0
#define AD5449_CTRL_LOADUPDATE(x)   (1 + 3 * (x))
#define AD5449_CTRL_READBACK(x)     (2 + 3 * (x))
#define AD5449_CTRL_LOAD(x)         (3 + 3 * (x))
#define AD5449_CTRL_UPDATEALL       7
#define AD5449_CTRL_LOADALL         8
#define AD5449_CTRL_DAISY_CHAIN     9
#define AD5449_CTRL_CLK_EDGE        10
#define AD5449_CTRL_CLR_ZERO        11
#define AD5449_CTRL_CLR_MID         12
#define AD5449_CTRL_REG             13

/* AD5449 channels */
#define AD5449_CH_A                 0
#define AD5449_CH_B                 1

/* Clear target scales */
#define AD5449_ZERO_SCALE            0
#define AD5449_MID_SCALE             1

/* Active clock edge */
#define AD5449_CLOCK_NEGEDGE         0
#define AD5449_CLOCK_POSEDGE         1

/* Daisy-Chain Control */
#define AD5449_DAISY_CHAIN_DIS       0
#define AD5449_DAISY_CHAIN_EN        1

/* AD5449_CTRL_REG definition */
#define AD5449_SDO_MASK          (3 << 10)
#define AD5449_DSY_MASK          (1 << 9)
#define AD5449_HCLR_MASK         (1 << 8)
#define AD5449_SCLK_MASK         (1 << 7)
#define AD5449_SDO_BIT           10
#define AD5449_DSY_BIT           9
#define AD5449_HCLR_BIT          8
#define AD5449_SCLK_BIT          7

/* AD5449 GPIO */
#define AD5449_LDAC_OUT             gpio_direction_output(dev->gpio_ldac,   \
			            GPIO_HIGH)
#define AD5449_LDAC_LOW             gpio_set_value(dev->gpio_ldac,          \
			            GPIO_LOW)
#define AD5449_LDAC_HIGH            gpio_set_value(dev->gpio_ldac,          \
			            GPIO_HIGH)

#define AD5449_CLR_OUT              gpio_direction_output(dev->gpio_clr,   \
			            GPIO_HIGH)
#define AD5449_CLR_LOW              gpio_set_value(dev->gpio_clr,          \
			            GPIO_LOW)
#define AD5449_CLR_HIGH             gpio_set_value(dev->gpio_clr,          \
			            GPIO_HIGH)

/* SDO Control Bits */
#define AD5449_SDO_FULL             0
#define AD5449_SDO_WEAK             1
#define AD5449_SDO_OPEN_DRAIN       2
#define AD5449_SDO_DISABLE          3

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize SPI and Initial Values for AD5449 Board. */
char AD5449_Init(ad5449_dev **device,
		 ad5449_init_param init_param);

/* Free the resources allocated by AD5449_Init(). */
int32_t AD5449_remove(ad5449_dev *dev);

/* Write to shift register via SPI. */
unsigned short AD5449_SetInputShiftReg(ad5449_dev *dev,
				       unsigned short command,
                                       unsigned short data);

/* Load and updates the selected DAC with a given value. */
void AD5449_LoadUpdateChannel(ad5449_dev *dev,
			      unsigned char channel,
			      unsigned short dacValue);

/* Load selected DAC input register with a given value. */
void AD5449_LoadChannel(ad5449_dev *dev,
			unsigned char channel,
			unsigned short dacValue);

/* Read from the selected DAC register. */
unsigned short AD5449_ReadbackChannel(ad5449_dev *dev,
				      unsigned char channel);

/* Update the DAC outputs (all channels). */
void AD5449_UpdateAll(ad5449_dev *dev);

/* Load the DAC input registers. */
void AD5449_LoadAll(ad5449_dev *dev,
		    short dacValue);

/* Set up the scale where to the output will be cleared on active CLR signal */
void AD5449_ClearScaleSetup(ad5449_dev *dev,
			    char type);

/* Enable/disable the Daisy-Chain mode */
void AD5449_DaisyChainSetup(ad5449_dev *dev,
			    char value);

/* Control the SDO output driver strength */
void AD5449_SDOControl(ad5449_dev *dev,
		       char controlBits);

/* Set up the active clock edge of the SPI interface */
void AD5449_SCLKSetup(ad5449_dev *dev,
		      char value);
