/***************************************************************************//**
 *   @file   ad_fmcjesdadc1_ebz.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2014(c) Analog Devices, Inc.
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
#include "platform_drivers.h"
#include "ad9517.h"
#include "ad9250.h"
#include "adc_core.h"
#include "xcvr_core.h"
#include "jesd_core.h"
#include "dmac_core.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define GPIO_JESD204_SYSREF		32

/***************************************************************************//**
* @brief main
*******************************************************************************/
int main(void)
{
	struct ad9250_dev		*ad9250_0_device;
	struct ad9250_dev		*ad9250_1_device;
	struct ad9517_dev		*ad9517_device;
	adc_core		ad9250_0_core;
	adc_core		ad9250_1_core;
	struct ad9250_init_param	ad9250_0_param;
	struct ad9250_init_param	ad9250_1_param;
	struct ad9517_init_param	ad9517_param;
	jesd_core		ad9250_jesd204;
	xcvr_core		ad9250_xcvr;
	dmac_core		ad9250_0_dma;
	dmac_core		ad9250_1_dma;
	dmac_xfer		rx_xfer_0;
	dmac_xfer		rx_xfer_1;

#ifdef XILINX
	ad9250_xcvr.base_address = XPAR_AXI_AD9250_XCVR_BASEADDR;
	ad9250_jesd204.base_address = XPAR_AXI_AD9250_JESD_RX_AXI_BASEADDR;
	ad9250_0_core.base_address = XPAR_AXI_AD9250_0_CORE_BASEADDR;
	ad9250_1_core.base_address = XPAR_AXI_AD9250_1_CORE_BASEADDR;

	ad9250_0_dma.base_address = XPAR_AXI_AD9250_0_DMA_BASEADDR;
	ad9250_1_dma.base_address = XPAR_AXI_AD9250_1_DMA_BASEADDR;
#endif

#ifdef ZYNQ
	rx_xfer_0.start_address = XPAR_DDR_MEM_BASEADDR + 0x800000;
	rx_xfer_1.start_address = XPAR_DDR_MEM_BASEADDR + 0x900000;
#endif

#ifdef MICROBLAZE
	rx_xfer_0.start_address = XPAR_AXI_DDR_CNTRL_BASEADDR + 0x800000;
	rx_xfer_1.start_address = XPAR_AXI_DDR_CNTRL_BASEADDR + 0x900000;
#endif

#ifdef ALTERA
	ad9250_xcvr.base_address = AD9250_JESD204_LINK_MANAGEMENT_BASE;
	ad9250_xcvr.dev.link_pll.base_address = AD9250_JESD204_LINK_PLL_RECONFIG_BASE;
	ad9250_0_core.base_address = AXI_AD9250_CORE_0_BASE;
	ad9250_0_core.base_address = AXI_AD9250_CORE_1_BASE;
	ad9250_jesd204.base_address = AD9250_JESD204_LINK_RECONFIG_BASE;

	ad9250_xcvr.dev.channel_pll[0].type = cmu_cdr_type;
	ad9250_xcvr.dev.channel_pll[0].base_address = AVL_ADXCFG_0_BASE;

	ad9250_0_dma.base_address = AXI_AD9250_DMA_0_BASE;
	ad9250_1_dma.base_address = AXI_AD9250_DMA_1_BASE;
	rx_xfer_0.start_address =  0x800000;
	rx_xfer_1.start_address =  0x900000;
#endif

	// SPI configuration

	ad9517_param.spi_init.chip_select = 0xe;
	ad9517_param.spi_init.cpha = 0;
	ad9517_param.spi_init.cpol = 0;
	ad9517_param.spi_init.type = ZYNQ_PS7_SPI;
	ad9250_0_param.spi_init.chip_select = 0xe;
	ad9250_0_param.spi_init.cpha = 0;
	ad9250_0_param.spi_init.cpol = 0;
	ad9250_0_param.spi_init.type = ZYNQ_PS7_SPI;
	ad9250_1_param.spi_init.chip_select = 0xe;
	ad9250_1_param.spi_init.cpha = 0;
	ad9250_1_param.spi_init.cpol = 0;
	ad9250_1_param.spi_init.type = ZYNQ_PS7_SPI;
	ad9250_0_param.id_no = 0x0;
	ad9250_1_param.id_no = 0x1;

	// ADC and receive path configuration
	ad9250_0_param.lane_rate_kbps = 4915200;
	ad9250_1_param.lane_rate_kbps = 4915200;

	xcvr_getconfig(&ad9250_xcvr);

	ad9250_xcvr.reconfig_bypass = 0;
	ad9250_xcvr.lane_rate_kbps = ad9250_0_param.lane_rate_kbps;
	ad9250_xcvr.ref_clock_khz = 245760;

	ad9250_jesd204.scramble_enable = 1;
	ad9250_jesd204.octets_per_frame = 1;
	ad9250_jesd204.frames_per_multiframe = 32;
	ad9250_jesd204.subclass_mode = 1;
	ad9250_jesd204.sysref_type = INTERN;
	ad9250_jesd204.sysref_gpio_pin = GPIO_JESD204_SYSREF;

	ad9250_0_core.no_of_channels = 2;
	ad9250_0_core.resolution = 14;

	ad9250_1_core.no_of_channels = 2;
	ad9250_1_core.resolution = 14;

	ad9250_0_dma.type = DMAC_RX;
	ad9250_0_dma.transfer = &rx_xfer_0;
	rx_xfer_0.id = 0;
	rx_xfer_0.no_of_samples = 32768;

	ad9250_1_dma.type = DMAC_RX;
	ad9250_1_dma.transfer = &rx_xfer_1;
	rx_xfer_1.id = 0;
	rx_xfer_1.no_of_samples = 32768;

	// set up clock generator
	ad9517_setup(&ad9517_device, ad9517_param);

	// set up the devices
	ad9250_setup(&ad9250_0_device, ad9250_0_param);
	ad9250_setup(&ad9250_1_device, ad9250_1_param);

	// generate SYSREF
	jesd_sysref_control(&ad9250_jesd204, 1);

	// set up the XCVR core
	xcvr_setup(&ad9250_xcvr);

	// set up the JESD core
	jesd_setup(&ad9250_jesd204);

	// JESD core status
	axi_jesd204_rx_status_read(&ad9250_jesd204);

	// set up interface core
	adc_setup(ad9250_0_core);
	adc_setup(ad9250_1_core);

	// PRBS test
	ad9250_test(ad9250_0_device, AD9250_TEST_PNLONG);
	if(adc_pn_mon(ad9250_0_core, ADC_PN23) == -1) {
		ad_printf("%s ad9250_0 - PN23 sequence mismatch!\n", __func__);
	};
	ad9250_test(ad9250_1_device, AD9250_TEST_PNLONG);
	if(adc_pn_mon(ad9250_1_core, ADC_PN23) == -1) {
		ad_printf("%s ad9250_1 - PN23 sequence mismatch!\n", __func__);
	};

	// set up ramp output
	ad9250_test(ad9250_0_device, AD9250_TEST_RAMP);
	ad9250_test(ad9250_1_device, AD9250_TEST_RAMP);

	// test the captured data
	if(!dmac_start_transaction(ad9250_0_dma)) {
		adc_ramp_test(ad9250_0_core, 1, 1024, rx_xfer_0.start_address);
	};
	if(!dmac_start_transaction(ad9250_1_dma)) {
		adc_ramp_test(ad9250_1_core, 1, 1024, rx_xfer_1.start_address);
	};

	// set up normal output
	ad9250_test(ad9250_0_device, AD9250_TEST_OFF);
	ad9250_test(ad9250_1_device, AD9250_TEST_OFF);

	// capture data with DMA

	if(!dmac_start_transaction(ad9250_0_dma)) {
		ad_printf("%s RX capture done!\n", __func__);
	};
	if(!dmac_start_transaction(ad9250_1_dma)) {
		ad_printf("%s RX capture done!\n", __func__);
	};

	ad9517_remove(ad9517_device);
	ad9250_remove(ad9250_0_device);
	ad9250_remove(ad9250_1_device);

	return 0;
}
