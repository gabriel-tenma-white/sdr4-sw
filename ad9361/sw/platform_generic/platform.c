/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "../util.h"
#include "platform.h"
#include <stdio.h>

#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>

typedef struct pollfd pollfd;
typedef int16_t s16;
typedef int32_t s32;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u8;
typedef uint64_t u64;

int ttyFD = -1;


void drainfd(int fd) {
	pollfd pfd;
	pfd.fd = fd;
	pfd.events = POLLIN;
	while(poll(&pfd,1,0)>0) {
		if(!(pfd.revents&POLLIN)) continue;
		char buf[4096];
		read(fd,buf,sizeof(buf));
	}
}

int writeAll(int fd,void* buf, int len) {
	u8* buf1=(u8*)buf;
	int off=0;
	int r;
	while(off<len) {
		if((r=write(fd,buf1+off,len-off))<=0) break;
		off+=r;
	}
	return off;
}

int readAll(int fd,void* buf, int len) {
	u8* buf1=(u8*)buf;
	int off=0;
	int r;
	while(off<len) {
		if((r=read(fd,buf1+off,len-off))<=0) break;
		off+=r;
	}
	return off;
}

void concat(u8* buf, int& index, u8* data, int len) {
	memcpy(buf+index, data, len);
	index += len;
}


void spiTransaction(u8* din, u8* dout, int bytes) {
	// bit pattern:
	// smp 0 0 0 0 sdi cs clk
	
	// pull down cs pin
	u8 buf[256] = {
		0b00000010,
		0b00000010,
		0b00000010,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000
	};
	int index = 7;
	
	// send data bits and sample dout bits
	for(int b=0;b<bytes;b++) {
		u8 byte = din[b];
		for(int i=0;i<8;i++) {
			u32 bit = byte>>7;
			u8 tmp[4] = {
				0b00000001,
				0b10000000		// MSB of 1 means to sample gpio inputs
			};
			tmp[0] |= bit<<2;
			tmp[1] |= bit<<2;
			tmp[2] |= bit<<2;
			tmp[3] |= bit<<2;
			concat(buf, index, tmp, sizeof(tmp));
			byte <<= 1;
		}
	}
	
	// release cs pin
	{
		u8 tmp[7] = {
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000010,
			0b00000010,
			0b00000010,
		};
		concat(buf, index, tmp, sizeof(tmp));
	}
	
	// write commands to tty device
	assert(writeAll(ttyFD, buf, index) == index);
	
	// read back data
	u8 buf2[bytes*8];
	assert(readAll(ttyFD,buf2,sizeof(buf2)) == (int)sizeof(buf2));
	for(int i=0;i<bytes;i++) {
		u8 byte=0;
		for(int j=0;j<8;j++) {
			u8 bit = (buf2[i*8+j] & 0b1000)?1:0;
			byte = (byte<<1) | bit;
		}
		dout[i] = byte;
	}
}


/***************************************************************************//**
 * @brief spi_init
*******************************************************************************/
int32_t spi_init(uint32_t device_id,
				 uint8_t  clk_pha,
				 uint8_t  clk_pol)
{
	ttyFD = open("/dev/ttyACM0",O_RDWR);
	if(ttyFD<0) {
		perror("open tty");
		return -1;
	}
	
	struct termios tc;
	
	/* Set TTY mode. */
	if (tcgetattr(ttyFD, &tc) < 0) {
		perror("tcgetattr");
		return 0;
	}
	tc.c_iflag &= ~(INLCR|IGNCR|ICRNL|IGNBRK|IUCLC|INPCK|ISTRIP|IXON|IXOFF|IXANY);
	tc.c_oflag &= ~OPOST;
	tc.c_cflag &= ~(CSIZE|CSTOPB|PARENB|PARODD|CRTSCTS);
	tc.c_cflag |= CS8 | CREAD | CLOCAL;
	tc.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG|IEXTEN);
	tc.c_cc[VMIN] = 1;
	tc.c_cc[VTIME] = 0;
	if (tcsetattr(ttyFD, TCSANOW, &tc) < 0) {
		perror("tcsetattr");
	}
	
	u8 buf[2] = {
		0b00000010,
		0b00000010,
	};
	assert(writeAll(ttyFD, buf, sizeof(buf)) == (int)sizeof(buf));
	
	usleep(10000);
	drainfd(ttyFD);
	return 0;
}

void spi_deinit() {
	u8 buf[2] = {
		0b00001010,
		0b00001010,
	};
	assert(writeAll(ttyFD, buf, sizeof(buf)) == (int)sizeof(buf));
}

/***************************************************************************//**
 * @brief spi_read
 * Perform a spi transaction of bytes_number total bytes, with sdi data supplied 
 * in data and sdo data also written back to data
*******************************************************************************/
int32_t spi_read(uint8_t *data,
				 uint8_t bytes_number)
{
	fprintf(stderr, "spi_read %d\n", bytes_number);
	spiTransaction(data, data, bytes_number);
	return 0;
}

/***************************************************************************//**
 * @brief spi_write_then_read
 * Perform a spi transaction of n_tx+n_rx bytes total, with the first n_tx bytes
 * sdi supplied from txbuf, and with the last n_rx bytes sdo written into rxbuf
*******************************************************************************/
int spi_write_then_read(struct spi_device *spi,
		const unsigned char *txbuf, unsigned n_tx,
		unsigned char *rxbuf, unsigned n_rx)
{
	//fprintf(stderr, "spi_write_then_read %d, %d\n", n_tx, n_rx);
	u8 buf[n_tx+n_rx];
	memset(buf,0,sizeof(buf));
	memcpy(buf, txbuf, n_tx);
	spiTransaction(buf, buf, n_tx+n_rx);
	memcpy(rxbuf, buf+n_tx, n_rx);
	return 0;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
void gpio_init(uint32_t device_id)
{

}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
void gpio_direction(uint8_t pin, uint8_t direction)
{

}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
bool gpio_is_valid(int number)
{
	return 0;
}

/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
void gpio_data(uint8_t pin, uint8_t data)
{

}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
void gpio_set_value(unsigned gpio, int value)
{

}

/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
void udelay(unsigned long usecs)
{

}

/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
void mdelay(unsigned long msecs)
{

}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long msleep_interruptible(unsigned int msecs)
{

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void axiadc_init(struct ad9361_rf_phy *phy)
{

}

/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{

}

/***************************************************************************//**
* @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_idelay_set
*******************************************************************************/
void axiadc_idelay_set(struct axiadc_state *st,
				unsigned lane, unsigned val)
{

}
