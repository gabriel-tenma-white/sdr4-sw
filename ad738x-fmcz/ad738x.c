/***************************************************************************//**
 *   @file   ad738x.c
 *   @brief  Implementation of AD738x Driver.
 *   @author SPopa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "spi_engine.h"
#include "ad738x.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/
/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_spi_reg_read(ad738x_dev *dev,
                            uint8_t reg_addr,
                            uint16_t *reg_data)
{
        int32_t ret;
        uint8_t buf[2];

        buf[0] = AD738X_REG_READ(reg_addr);
        buf[1] = 0x00;

        ret = spi_write_and_read(dev->spi_desc, buf, 2);
        *reg_data = (buf[0] << 8) | buf[1];

        return ret;
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_spi_reg_write(ad738x_dev *dev,
		 	 	 	 	 	 uint8_t reg_addr,
							 uint16_t reg_data)
{
	uint8_t buf[2];
	int32_t ret;

    buf[0] = AD738X_REG_WRITE(reg_addr) | ((reg_data & 0xF00) >> 8);
    buf[1] = reg_data & 0xFFF;

    ret = spi_write_and_read(dev->spi_desc, buf, 2);

	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_spi_write_mask(ad738x_dev *dev,
							  uint8_t reg_addr,
							  uint32_t mask,
							  uint16_t data)
{
	uint16_t reg_data;
	int32_t ret;

	ret = ad738x_spi_reg_read(dev, reg_addr, &reg_data);
	reg_data &= ~mask;
	reg_data |= data;
	ret |= ad738x_spi_reg_write(dev, reg_addr, reg_data);

	return ret;
}

/**
 * Read conversion result from device.
 * @param dev - The device structure.
 * @param adc_data - The conversion result data
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_spi_single_conversion(ad738x_dev *dev,
								 	 uint16_t *adc_data)
{
	uint8_t buf[4];
	uint8_t rx_buf_len;
	int32_t ret;

    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;

    /* Conversion data is 2 bytes long */
    rx_buf_len = 2 * dev->conv_mode + 2;
    ret = spi_write_and_read(dev->spi_desc, buf, rx_buf_len);

    /*
     *  Conversion data is 16 bits long in 1-wire mode and
     *  32 bits in 2-wire mode
     */
    adc_data[0] =  (buf[0] << 8) | buf[1];
    adc_data[1] =  (buf[2] << 8) | buf[3];

    return ret;
}

/**
 * Select if ADC A and ADC B output on both SDOA and SDOB lines
 * (two wire mode) or only on on the SDOA line
 * @param dev - The device structure.
 * @param mode - The conversion output mode.
 * 					Accepted values: TWO_WIRE_MODE
 *									 ONE_WIRE_MODE
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_set_conversion_mode(ad738x_dev *dev,
								   ad738x_conv_mode mode)
{
	int32_t ret;

	ret = ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG2,
								AD738X_CONFIG2_SDO2_MSK,
								AD738X_CONFIG2_SDO2(mode));

	dev->conv_mode = mode;

	return ret;
}

/**
 * Device reset over SPI.
 * @param dev - The device structure.
 * @param reset - Type of reset.
 * 					Accepted values: SOFT_RESET
 * 									 HARD_RESET
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_reset(ad738x_dev *dev,
					 ad738x_reset_type reset)
{
	int32_t ret;
	uint32_t val = ((reset == HARD_RESET) ? 0xFF : 0x3C);

	ret = ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG2,
								AD738X_CONFIG2_RESET_MSK,
								AD738X_CONFIG2_RESET(val));

	return ret;
}

/**
 * Sets the oversampling mode in the device (os_mode)
 * Sets the oversampling ratio (osr)
 * Sets the size of the conversion result data (res)
 * @param dev - The device structure.
 * @param os_mode - accepted values: NORMAL_OS_MODE
 * 									 ROLLING_OS_MODE
 * @param os_ratio - accepted values: OSR_DISABLED
 * 									  OSR_X2
 * 									  OSR_X4
 * 									  OSR_X8
 * 									  OSR_X16
 * 									  OSR_X32
 * @param res - accepted values: RES_16_BIT
 * 								 RES_18_BIT
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_oversampling_config(ad738x_dev *dev,
								   ad738x_os_mode os_mode,
								   ad738x_os_ratio os_ratio,
								   ad738x_resolution res)
{
	int32_t ret;
	uint16_t reg_data;

	ret = ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG1,
								AD738X_CONFIG1_OS_MODE_MSK,
								AD738X_CONFIG1_OS_MODE(os_mode));

	ret |= ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG1,
								AD738X_CONFIG1_OSR_MSK,
								AD738X_CONFIG1_OSR(os_ratio));

	ret |= ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG1,
								AD738X_CONFIG1_RES_MSK,
								AD738X_CONFIG1_RES(res));

	ret |= ad738x_spi_reg_read(dev, AD738X_REG_CONFIG1, &reg_data);
	dev->resolution = (reg_data & AD738X_CONFIG1_RES_MSK) >> 2;

	return ret;
}

/**
 * Device power down.
 * @param dev - The device structure.
 * @param pmode - Type of power mode
 * 					Accepted values: NORMAL_PWDM
 * 									 FULL_PWDM
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_power_down_mode(ad738x_dev *dev,
							   ad738x_pwd_mode pmode)
{
	int32_t ret;

	ret = ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG1,
								AD738X_CONFIG1_PMODE_MSK,
								AD738X_CONFIG1_PMODE(pmode));

	return ret;
}

/**
 * Enable internal or external reference
 * @param dev - The device structure.
 * @param ref_sel - Type of reference
 * 					Accepted values: INT_REF
 * 									 EXT_REF
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_reference_sel(ad738x_dev *dev,
							 ad738x_ref_sel ref_sel)
{
	int32_t ret;

	ret = ad738x_spi_write_mask(dev,
								AD738X_REG_CONFIG1,
								AD738X_CONFIG1_REFSEL_MSK,
								AD738X_CONFIG1_REFSEL(ref_sel));

	return ret;
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 					   parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad738x_init(ad738x_dev **device,
                    ad738x_init_param init_param)
{
        ad738x_dev *dev;
        int32_t ret;

        dev = (ad738x_dev *)malloc(sizeof(*dev));
        if (!dev)
                return -1;

        ret = spi_init(&dev->spi_desc, init_param.spi_init);

        ret |= ad738x_reset(dev, HARD_RESET);
        mdelay(1000);
        /* 1-wire or 2-wire mode */
        ret |= ad738x_set_conversion_mode(dev, init_param.conv_mode);
        /* Set internal or external reference */
        ret |= ad738x_reference_sel(dev, init_param.ref_sel);

        *device = dev;

        if (!ret)
                printf("ad738x successfully initialized\n");
        mdelay(1000);

        return ret;
}

/***************************************************************************//**
 * @brief Free the resources allocated by ad738x_init().
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
*******************************************************************************/
int32_t ad738x_remove(ad738x_dev *dev)
{
        int32_t ret;

        ret = spi_remove(dev->spi_desc);

        free(dev);

        return ret;
}
