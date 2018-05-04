/***************************************************************************//**
 *   @file   AD7303.c
 *   @brief  Implementation of AD7303 Driver.
 *   @author Mihai Bancisor(Mihai.Bancisor@analog.com)
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
********************************************************************************
*   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "platform_drivers.h"
#include "AD7303.h"           // AD7303 definitions.

/***************************************************************************//**
 * @brief Initializes SPI communication.
 *
 * @param device     - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return Result of the initialization procedure.
 *            Example: -1 - SPI peripheral was not initialized.
 *                      0 - SPI peripheral is initialized.
*******************************************************************************/
char AD7303_Init(ad7303_dev **device,
		 ad7303_init_param init_param)
{
	ad7303_dev *dev;
    char status;

	dev = (ad7303_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	status = spi_init(&dev->spi_desc, init_param.spi_init);

	*device = dev;

    return status;
}

/***************************************************************************//**
 * @brief Free the resources allocated by AD7303_Init().
 *
 * @param dev - The device structure.
 *
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t AD7303_remove(ad7303_dev *dev)
{
	int32_t ret;

	ret = spi_remove(dev->spi_desc);

	free(dev);

	return ret;
}

/***************************************************************************//**
 * @brief Sends data to AD7303.
 *
 * @param dev        - The device structure.
 * @param controlReg - Value of control register.
 *                     Example:
 *                     AD7303_INT | AD7303_LDAC | AD7303_A  - enables internal
 *                     reference and loads DAC A input register from shift
 *                     register and updates both DAC A and DAC B DAC registers.
 * @param dataReg    - Value of data register.
 *
 * @return None.
*******************************************************************************/
void AD7303_Write(ad7303_dev *dev,
		  unsigned char controlReg,
		  unsigned char dataReg)
{
    static unsigned char writeData[2] = {0, 0};

    writeData[0] = controlReg;
    writeData[1] = dataReg;
	spi_write_and_read(dev->spi_desc,
			   writeData,
			   2);
}
