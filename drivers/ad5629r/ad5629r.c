/**************************************************************************//**
*   @file   ad5629r.c
*   @brief  Implementation of ad5629r Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "platform_drivers.h"
#include "ad5629r.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
static const ad5629r_chip_info chip_info[] = {
    [ID_AD5629R] = {
        .resolution = 12,
        .communication = com_i2c,
    },
    [ID_AD5669R] = {
        .resolution = 16,
        .communication = com_i2c,
    },
    [ID_AD5668] = {
        .resolution = 16,
        .communication = com_spi,
    },
    [ID_AD5648] = {
        .resolution = 14,
        .communication = com_spi,
    },
    [ID_AD5628] = {
        .resolution = 12,
        .communication = com_spi,
    }
};

/******************************************************************************/
/************************** Functions Definitions *****************************/
/******************************************************************************/


/***************************************************************************//**
 * @brief Initializes the communication with the device.
 *
 * @param devAddr - AD5629R device address.
 *                  Example: AD5629R_I2C_ADDR_0 - A0_Pin=High;
 *                           AD5629R_I2C_ADDR_1 - A0_Pin=NC;
 *                           AD5629R_I2C_ADDR_2 - A0_Pin=Low.
 *        device  - supported devices.
 *                  Example: AD5629R, AD5669R, AD5668, AD5648, AD5628.
 * @return status - Result of the initialization procedure.
 *                  Example:  0 - if initialization was successful;
 *                           -1 - if initialization was unsuccessful.
*******************************************************************************/
char AD5629R_Init(ad5629r_dev **device,
		  ad5629r_init_param init_param)
{
	ad5629r_dev *dev;
    char status;

	dev = (ad5629r_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

    dev->act_device = init_param.act_device;

    if (chip_info[dev->act_device].communication == com_spi)
    {
	    status = spi_init(&dev->spi_desc, init_param.spi_init);
    }
    else
    {
	    status = i2c_init(&dev->i2c_desc, init_param.i2c_init);
    }

	status |= gpio_get(&dev->gpio_ldac, init_param.gpio_ldac);
	status |= gpio_get(&dev->gpio_clr, init_param.gpio_clr);

    AD5629R_LDAC_OUT;
    AD5629R_LDAC_LOW;
    AD5629R_CLR_OUT;
    AD5629R_CLR_HIGH;

	*device = dev;

    return status;
}

/***************************************************************************//**
 * @brief Free the resources allocated by AD5629R_Init().
 *
 * @param dev - The device structure.
 *
 * @return ret - The result of the remove procedure.
*******************************************************************************/
int32_t AD5629R_remove(ad5629r_dev *dev)
{
	int32_t ret;

	if (chip_info[dev->act_device].communication == com_spi)
		ret = spi_remove(dev->spi_desc);
	else
		ret = i2c_remove(dev->i2c_desc);

	if (dev->gpio_ldac)
		ret |= gpio_remove(dev->gpio_ldac);

	if (dev->gpio_clr)
		ret |= gpio_remove(dev->gpio_clr);

	free(dev);

	return ret;
}

/**************************************************************************//**
 * @brief Write to input register and read from output register via SPI.
 *
 * @param   dev      - The device structure.
 *          function - command control bits;
 *          dacN     - address of selected DAC;
 *          data     - data to be written in register.
 *
 * @return  readBack - value read from register.
******************************************************************************/
void AD5629R_SetCtrl(ad5629r_dev *dev,
		     unsigned char function,
                     unsigned char dacN,
                     unsigned long data)
{
    unsigned char  dataBuff[4]   = {0, 0, 0, 0};

    if(chip_info[dev->act_device].communication == com_spi)
    {
        data = data & 0xFFFFF;

        dataBuff[0] = function;
        dataBuff[1] = (dacN << 4) | ((0xF0000 & data) >> 12);
        dataBuff[2] = (0xFF00 & data) >> 8;
        dataBuff[3] = (0xFF & data);

	    spi_write_and_read(dev->spi_desc,
			       dataBuff,
			       4);
    }
    else
    {
        if (chip_info[dev->act_device].communication == com_i2c)
        {
            dataBuff[0] = (function << 4) | dacN;
            dataBuff[1] = (data & 0xFF00) >> 8;
            dataBuff[2] = (data & 0x00FF) >> 0;

		i2c_write(dev->i2c_desc,
			  dataBuff,
			  3,
			  1);
        }
    }
}

/**************************************************************************//**
 * @brief Write to input register and read from output register via SPI.
 *
 * @param   dev      - The device structure.
 *          function - command control bits.
 *          dacN     - address of selected DAC;
 *          dacValue - data to be written in input register.
 *
 * @return  readBack - value read from register.
******************************************************************************/
void AD5629R_SetInputReg(ad5629r_dev *dev,
			 unsigned char function,
                         unsigned char dacN,
                         unsigned short dacValue)
{
    unsigned char  dataBuff[4]   = {0, 0, 0, 0};

    dacValue = dacValue << (MAX_RESOLUTION -
               chip_info[dev->act_device].resolution);

    if(chip_info[dev->act_device].communication == com_spi)
    {
        dacValue = dacValue & 0xFFFF;

        dataBuff[0] = function;
        dataBuff[1] = (dacN << 4) | ((0xF000 & dacValue) >> 12);
        dataBuff[2] = (0xFF0 & dacValue) >> 4;
        dataBuff[3] = (0xF & dacValue) << 4;

	    spi_write_and_read(dev->spi_desc,
			       dataBuff,
			       4);
    }
    else
    {
        dataBuff[0] = (function << 4) | dacN;
        dataBuff[1] = (dacValue & 0xFF00) >> 8;
        dataBuff[2] = (dacValue & 0x00FF) >> 0;

	    i2c_write(dev->i2c_desc,
			  dataBuff,
			  3,
			  1);
    }
}

/***************************************************************************//**
 * @brief Writes a value to Input Register N of selected DAC channel.
 *
 * @param dev      - The device structure.
 *        dacValue - Value to be written in register;
 *        dacN     - Address of selected DAC.
 *
 * @return none.
*******************************************************************************/
void AD5629R_WriteRegN(ad5629r_dev *dev,
		       unsigned char dacN,
		       unsigned short dacValue)
{
    AD5629R_SetInputReg(dev,
			AD5629R_WRITE_N,
			dacN,
			dacValue);
}

/***************************************************************************//**
 * @brief Updates selected DAC register.
 *
 * @param dev  - The device structure.
 *        dacN - Address of selected DAC.
 *
 * @return none.
*******************************************************************************/
void AD5629R_UpdateDacN(ad5629r_dev *dev,
			unsigned char dacN)
{
    AD5629R_SetInputReg(dev,
			AD5629R_UPDATE_N,
			dacN,
			0x0);
}

/***************************************************************************//**
 * @brief Writes a value to Input Register N of selected DAC channel, then
 *        updates all.
 *
 * @param dev      - The device structure.
 *        dacValue - Value to be written in register;
 *        dacN     - Address of selected DAC.
 *
 * @return none.
*******************************************************************************/
void AD5629R_WriteRegNUpdateAll(ad5629r_dev *dev,
				unsigned char dacN,
				unsigned short dacValue)
{
    AD5629R_SetInputReg(dev,
			AD5629R_WRITE_N_UPDATE_ALL,
			dacN,
			dacValue);
}

/***************************************************************************//**
 * @brief Writes a value to Input Register N and updates the respective DAC
 *        channel.
 *
 * @param dev      - The device structure.
 *        dacValue - Value to be written in register;
 *        dacN     - Address of selected DAC.
 *
 * @return none.
*******************************************************************************/
void AD5629R_WriteRegNUpdateN(ad5629r_dev *dev,
			      unsigned char dacN,
			      unsigned short dacValue)
{
    AD5629R_SetInputReg(dev,
			AD5629R_WRITE_N_UPDATE_N,
			dacN,
			dacValue);
}

/***************************************************************************//**
 * @brief Sets the power mode for one or more selected DAC channels.
 *
 * @param dev    - The device structure.
 *        dacSel - a byte where each bit is corresponding to a DAC; when a bit is
 *                 set to 1, the corresponding DAC is selected.
 *                 Example: DAC_A_SEL - the selected DAC is DAC A;
 *                          DAC_D_SEL | DAC_F_SEL | DAC_H_SEL - the selected DACs
 *                          are: DAC D, DAC F and DAC H.
 *        mode   - the desired power mode to be set.
 *                 Example: PWR_NORMAL      - normal operation;
 *                          PWR_1K_TO_GND   - 1KOhm to GND;
 *                          PWR_100K_TO_GND - 100KOhms to GND;
 *                          PWR_3_STATE     - three-state.
 *
 * @return none.
*******************************************************************************/
void AD5629R_SetPowerMode(ad5629r_dev *dev,
			  unsigned char dacSel,
			  unsigned char mode)
{
    unsigned long data = 0;

    data = (mode << 8) | dacSel;

    AD5629R_SetCtrl(dev,
		    AD5629R_POWER,
		    0x0,
		    data);
}

/***************************************************************************//**
 * @brief Loads the Clear Code Register with a certain value.
 *
 * @param dev        - The device structure.
 *        clearValue - the value to be set in all DAC registers after a clear
 *                     operation.
 *                     Example: CLR_TO_ZEROSCALE - clears to 0x0;
 *                              CLR_TO_MIDSCALE  - clears to 0x8000;
 *                              CLR_TO_FULLSCALE - clears to 0xFFFF;
 *                              CLR_NOOP         - no operation.
 *
 * @return none.
*******************************************************************************/
void AD5629R_LoadClearCodeReg(ad5629r_dev *dev,
			      unsigned char clearValue)
{
    unsigned long data = 0;

    data = (unsigned long)clearValue;

    AD5629R_SetCtrl(dev,
		    AD5629R_LOAD_CLEAR_REG,
		    0x0,
		    data);
}

/***************************************************************************//**
 * @brief Loads the LDAC register with a certain value.
 *
 * @param dev    - The device structure.
 *        dacSel - a byte where each bit is corresponding to a DAC; when a bit is
 *                 set to 1, the corresponding DAC is selected to override LDAC
 *                 pin.
 *                 Example: DAC_A_SEL - the selected DAC to override the LDAC pin
 *                                      is DAC A;
 *                          DAC_D_SEL | DAC_F_SEL | DAC_H_SEL - the selected DACs
 *                          to override the LDAC pin are: DAC D, DAC F and DAC H.
 *
 * @return none.
*******************************************************************************/
void AD5629R_LoadLdacReg(ad5629r_dev *dev,
			 unsigned char dacSel)
{
    unsigned long data = 0;

    data = (unsigned long)dacSel;

    AD5629R_SetCtrl(dev,
		    AD5629R_LOAD_LDAC_REG,
		    0x0,
		    data);
}

/***************************************************************************//**
 * @brief Makes a power-on reset.
 *
 * @param dev - The device structure.
 *
 * @return none.
*******************************************************************************/
void AD5629R_Reset(ad5629r_dev *dev)
{
    AD5629R_SetCtrl(dev,
		    AD5629R_RESET,
		    0x0,
		    0x0);
}

/***************************************************************************//**
 * @brief Turns on/off the internal reference.
 *
 * @param dev    - The device structure.
 *        status - the status of internal reference.
 *                 Example: REF_ON  - the reference is on;
 *                          REF_OFF - the reference is off.
 *
 * @return none.
*******************************************************************************/
void AD5629R_SetRef(ad5629r_dev *dev,
		    unsigned char status)
{
    AD5629R_SetCtrl(dev,
		    AD5629R_REFERENCE,
		    0x0,
		    (unsigned long)status);
}


