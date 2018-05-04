/***************************************************************************//**
 *   @file   ADF7023.c
 *   @brief  Implementation of ADF7023 Driver.
 *   @author DBogdan (Dragos.Bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#include "adf7023_config.h"
#include "adf7023.h"

/******************************************************************************/
/*************************** Macros Definitions *******************************/
/******************************************************************************/
#define ADF7023_CS_ASSERT   gpio_set_value(&dev->gpio_dev, \
			    dev->gpio_cs,                  \
			    GPIO_LOW)
#define ADF7023_CS_DEASSERT gpio_set_value(&dev->gpio_dev, \
			    dev->gpio_cs,                  \
			    GPIO_HIGH)

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/


/***************************************************************************//**
 * @brief Transfers one byte of data.
 *
 * @param dev - The device structure.
 * @param write_byte - Write data.
 * @param read_byte - Read data.
 *
 * @return None.
*******************************************************************************/
void adf7023_write_read_byte(adf7023_dev *dev,
			     uint8_t write_byte,
			     uint8_t* read_byte)
{
	uint8_t data = 0;

	data = write_byte;
	spi_write_and_read(&dev->spi_dev,
			   &data,
			   1);
	if (read_byte)
		*read_byte = data;
}

/***************************************************************************//**
 * @brief Initializes the ADF7023.
 *
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 		       parameters.
 *
 * @return ret - Result of the initialization procedure.
 *               Example: 0 - if initialization was successful;
 *                        -1 - if initialization was unsuccessful.
*******************************************************************************/
int32_t adf7023_init(adf7023_dev **device,
		     adf7023_init_param init_param)
{
	adf7023_dev *dev;
	uint8_t  miso    = 0;
	uint16_t timeout = 0;
	uint8_t  status  = 0;
	int32_t ret = 0;

	dev = (adf7023_dev *)malloc(sizeof(*dev));
	if (!dev)
		return -1;

	/* SPI */
	dev->spi_dev.type = init_param.spi_type;
	dev->spi_dev.id = init_param.spi_id;
	dev->spi_dev.max_speed_hz = init_param.spi_max_speed_hz;
	dev->spi_dev.mode = init_param.spi_mode;
	dev->spi_dev.chip_select = init_param.spi_chip_select;
	ret = spi_init(&dev->spi_dev);

	/* GPIO */
	dev->gpio_dev.id = init_param.gpio_id;
	dev->gpio_dev.type = init_param.gpio_type;
	ret |= gpio_init(&dev->gpio_dev);

	dev->gpio_cs = init_param.gpio_cs;
	dev->gpio_miso = init_param.gpio_miso;

	dev->adf7023_bbram_current = adf7023_bbram_default;

	if (dev->gpio_cs >= 0) {
		ret |= gpio_set_direction(&dev->gpio_dev,
					  dev->gpio_cs,
					  GPIO_OUT);
		ret |= gpio_set_value(&dev->gpio_dev,
				      dev->gpio_cs,
				      GPIO_HIGH);
	}

	ADF7023_CS_ASSERT;

	while ((miso == 0) && (timeout < 1000)) {
		gpio_get_value(&dev->gpio_dev, dev->gpio_miso, &miso);
		timeout++;
	}
	if (timeout == 1000)
		ret = -1;

	while(!(status & STATUS_CMD_READY))
		adf7023_get_status(dev, &status);

	adf7023_set_ram(dev, 0x100, 64, (uint8_t*)&dev->adf7023_bbram_current);
	adf7023_set_command(dev, CMD_CONFIG_DEV);

	*device = dev;

	return ret;
}

/***************************************************************************//**
 * @brief Reads the status word of the ADF7023.
 *
 * @param dev - The device structure.
 * @param status - Status word.
 *
 * @return None.
*******************************************************************************/
void adf7023_get_status(adf7023_dev *dev,
			uint8_t* status)
{
	ADF7023_CS_ASSERT;
	adf7023_write_read_byte(dev, SPI_NOP, 0);
	adf7023_write_read_byte(dev, SPI_NOP, status);
	ADF7023_CS_DEASSERT;
}

/***************************************************************************//**
 * @brief Initiates a command.
 *
 * @param dev - The device structure.
 * @param command - Command.
 *
 * @return None.
*******************************************************************************/
void adf7023_set_command(adf7023_dev *dev,
			 uint8_t command)
{
	ADF7023_CS_ASSERT;
	adf7023_write_read_byte(dev, command, 0);
	ADF7023_CS_DEASSERT;
}

/***************************************************************************//**
 * @brief Sets a FW state and waits until the device enters in that state.
 *
 * @param dev - The device structure.
 * @param fw_state - FW state.
 *
 * @return None.
*******************************************************************************/
void adf7023_set_fw_state(adf7023_dev *dev,
			  uint8_t fw_state)
{
	uint8_t status = 0;

	switch(fw_state) {
	case FW_STATE_PHY_OFF:
		adf7023_set_command(dev, CMD_PHY_OFF);
		break;
	case FW_STATE_PHY_ON:
		adf7023_set_command(dev, CMD_PHY_ON);
		break;
	case FW_STATE_PHY_RX:
		adf7023_set_command(dev, CMD_PHY_RX);
		break;
	case FW_STATE_PHY_TX:
		adf7023_set_command(dev, CMD_PHY_TX);
		break;
	default:
		adf7023_set_command(dev, CMD_PHY_SLEEP);
	}
	while((status & STATUS_FW_STATE) != fw_state) {
		adf7023_get_status(dev, &status);
	}
}

/***************************************************************************//**
 * @brief Reads data from the RAM.
 *
 * @param dev - The device structure.
 * @param address - Start address.
 * @param length - Number of bytes to write.
 * @param data - Read buffer.
 *
 * @return None.
*******************************************************************************/
void adf7023_get_ram(adf7023_dev *dev,
		     uint32_t address,
		     uint32_t length,
		     uint8_t* data)
{
	ADF7023_CS_ASSERT;
	adf7023_write_read_byte(dev, SPI_MEM_RD | ((address & 0x700) >> 8), 0);
	adf7023_write_read_byte(dev, address & 0xFF, 0);
	adf7023_write_read_byte(dev, SPI_NOP, 0);
	while(length--) {
		adf7023_write_read_byte(dev, SPI_NOP, data++);
	}
	ADF7023_CS_DEASSERT;
}

/***************************************************************************//**
 * @brief Writes data to RAM.
 *
 * @param dev - The device structure.
 * @param address - Start address.
 * @param length - Number of bytes to write.
 * @param data - Write buffer.
 *
 * @return None.
*******************************************************************************/
void adf7023_set_ram(adf7023_dev *dev,
		     uint32_t address,
		     uint32_t length,
		     uint8_t* data)
{
	ADF7023_CS_ASSERT;
	adf7023_write_read_byte(dev, SPI_MEM_WR | ((address & 0x700) >> 8), 0);
	adf7023_write_read_byte(dev, address & 0xFF, 0);
	while(length--) {
		adf7023_write_read_byte(dev, *(data++), 0);
	}
	ADF7023_CS_DEASSERT;
}

/***************************************************************************//**
 * @brief Receives one packet.
 *
 * @param dev - The device structure.
 * @param packet - Data buffer.
 * @param length - Number of received bytes.
 *
 * @return None.
*******************************************************************************/
void adf7023_receive_packet(adf7023_dev *dev,
			    uint8_t* packet,
			    uint8_t* length)
{
	uint8_t interruptReg = 0;

	adf7023_set_fw_state(dev, FW_STATE_PHY_ON);
	adf7023_set_fw_state(dev, FW_STATE_PHY_RX);
	while(!(interruptReg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_CRC_CORRECT)) {
		adf7023_get_ram(dev, MCR_REG_INTERRUPT_SOURCE_0,
				0x1,
				&interruptReg);
	}
	adf7023_set_ram(dev, MCR_REG_INTERRUPT_SOURCE_0,
			0x1,
			&interruptReg);
	adf7023_get_ram(dev, 0x10, 1, length);
	adf7023_get_ram(dev, 0x12, *length - 2, packet);
}

/***************************************************************************//**
 * @brief Transmits one packet.
 *
 * @param dev - The device structure.
 * @param packet - Data buffer.
 * @param length - Number of bytes to transmit.
 *
 * @return None.
*******************************************************************************/
void adf7023_transmit_packet(adf7023_dev *dev,
			     uint8_t* packet,
			     uint8_t length)
{
	uint8_t interruptReg = 0;
	uint8_t header[2]    = {0, 0};

	header[0] = 2 + length;
	header[1] = dev->adf7023_bbram_current.address_match_offset;
	adf7023_set_ram(dev, 0x10, 2, header);
	adf7023_set_ram(dev, 0x12, length, packet);
	adf7023_set_fw_state(dev, FW_STATE_PHY_ON);
	adf7023_set_fw_state(dev, FW_STATE_PHY_TX);
	while(!(interruptReg & BBRAM_INTERRUPT_MASK_0_INTERRUPT_TX_EOF)) {
		adf7023_get_ram(dev, MCR_REG_INTERRUPT_SOURCE_0,
				0x1,
				&interruptReg);
	}
}

/***************************************************************************//**
 * @brief Sets the channel frequency.
 *
 * @param dev - The device structure.
 * @param chFreq - Channel frequency.
 *
 * @return None.
*******************************************************************************/
void adf7023_set_channel_frequency(adf7023_dev *dev,
				   uint32_t chFreq)
{
	chFreq = (uint32_t)(((float)chFreq / 26000000) * 65535);
	dev->adf7023_bbram_current.channel_freq0 = (chFreq & 0x0000FF) >> 0;
	dev->adf7023_bbram_current.channel_freq1 = (chFreq & 0x00FF00) >> 8;
	dev->adf7023_bbram_current.channel_freq2 = (chFreq & 0xFF0000) >> 16;
	adf7023_set_ram(dev, 0x100, 64, (uint8_t*)&dev->adf7023_bbram_current);
}

/***************************************************************************//**
 * @brief Sets the data rate.
 *
 * @param dev - The device structure.
 * @param data_rate - Data rate.
 *
 * @return None.
*******************************************************************************/
void ADF7023_SetDataRate(adf7023_dev *dev,
			 uint32_t data_rate)
{
	data_rate = (uint32_t)(data_rate / 100);
	dev->adf7023_bbram_current.radio_cfg0 =
		BBRAM_RADIO_CFG_0_DATA_RATE_7_0((data_rate & 0x00FF) >> 0);
	dev->adf7023_bbram_current.radio_cfg1 &= ~BBRAM_RADIO_CFG_1_DATA_RATE_11_8(0xF);
	dev->adf7023_bbram_current.radio_cfg1 |=
		BBRAM_RADIO_CFG_1_DATA_RATE_11_8((data_rate & 0x0F00) >> 8);
	adf7023_set_ram(dev, 0x100, 64, (uint8_t*)&dev->adf7023_bbram_current);
	adf7023_set_fw_state(dev, FW_STATE_PHY_OFF);
	adf7023_set_command(dev, CMD_CONFIG_DEV);
}

/***************************************************************************//**
 * @brief Sets the frequency deviation.
 *
 * @param dev - The device structure.
 * @param freq_dev - Frequency deviation.
 *
 * @return None.
*******************************************************************************/
void ADF7023_SetFrequencyDeviation(adf7023_dev *dev,
				   uint32_t freq_dev)
{
	freq_dev = (uint32_t)(freq_dev / 100);
	dev->adf7023_bbram_current.radio_cfg1 &=
		~BBRAM_RADIO_CFG_1_FREQ_DEVIATION_11_8(0xF);
	dev->adf7023_bbram_current.radio_cfg1 |=
		BBRAM_RADIO_CFG_1_FREQ_DEVIATION_11_8((freq_dev & 0x0F00) >> 8);
	dev->adf7023_bbram_current.radio_cfg2 =
		BBRAM_RADIO_CFG_2_FREQ_DEVIATION_7_0((freq_dev & 0x00FF) >> 0);
	adf7023_set_ram(dev, 0x100, 64, (uint8_t*)&dev->adf7023_bbram_current);
	adf7023_set_fw_state(dev, FW_STATE_PHY_OFF);
	adf7023_set_command(dev, CMD_CONFIG_DEV);
}
