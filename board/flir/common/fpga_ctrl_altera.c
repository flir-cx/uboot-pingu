// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include "fpga_ctrl.h"

#include <common.h>

#include <asm/gpio.h>
#include <log.h>
#include <linux/delay.h>

//FPGA Configuration is documented in
// Altera: CV-52007 (2017.09.19)
//         Cyclone Device Handbook Volume 1, Chapter 7 Configuration
//
// EC501 FPGA: Altera 5CGXF7B6M157
// Configuration MSEL[4:0] = [10011] -- Active serial mode
//pin             CPU config                                     config fnctn
//SPI1_SCLK_FPGA  hiZ                                            disable_spi_bus
//SPI1_CS1_n      hiZ                                            disable_spi_bus
//SPI1_MOSI       hiZ                                            disable_spi_bus
//SPI1_MISO       hiZ                                            disable_spi_bus
//FPGA_CE_n       output, must be low during config,             power_up_fpga
//                set high to release spi
//CONFIG_n        output, config triggered by L->H transition    power_up_fpga
//STATUS_n        input                                          power_up_fpga
//CONF_DONE       input                                          power_up_fpga

static int altera_fpga_request_pinctl(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret += gpio_request(fpga->pins.config_n, "FPGA Config");
	ret += gpio_request(fpga->pins.status_n, "FPGA Status");
	ret += gpio_request(fpga->pins.ce, "FPGA CE");
	ret += gpio_request(fpga->pins.done, "FPGA CONF_DONE");

	if (ret) {
		log_err("%s: gpio request failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int altera_fpga_release_pinctl(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret += gpio_free(fpga->pins.config_n);
	ret += gpio_free(fpga->pins.status_n);
	ret += gpio_free(fpga->pins.ce);
	ret += gpio_free(fpga->pins.done);

	if (ret) {
		log_err("%s: gpio free failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int altera_fpga_enable(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret = gpio_direction_output(fpga->pins.ce, 0);
	udelay(10);

	if (ret)
		log_err("%s: gpio set direction failed\n", __func__);

	return ret;
}

static int altera_fpga_disable(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret = gpio_direction_output(fpga->pins.ce, 1);
	udelay(10);

	if (ret)
		log_err("%s: gpio set direction failed\n", __func__);

	return ret;
}

static int altera_fpga_hold_in_config(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);

	ret += gpio_direction_output(fpga->pins.config_n, 0);
	ret += gpio_direction_input(fpga->pins.done);
	ret += gpio_direction_input(fpga->pins.status_n);

	if (ret) {
		log_err("%s: gpio set direction failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int altera_fpga_start_config(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);

	ret += gpio_direction_output(fpga->pins.config_n, 0);
	ret += gpio_direction_output(fpga->pins.ce, 0);
	mdelay(10);
	ret += gpio_direction_output(fpga->pins.config_n, 1);

	if (ret) {
		log_err("%s: gpio set direction failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int altera_fpga_poll_config_status(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	// Check if configuration is done
	ret = gpio_get_value(fpga->pins.done);
	debug("%s: done = %d\n", __func__, ret);
	if (ret == 1)
		return 1;

	return 0;
}

void fpga_set_ops(struct fpga_ctrl *ctrl)
{
	debug("%s: Altera fpga chosen\n", __func__);
	ctrl->ops.fpga_request_pinctl = altera_fpga_request_pinctl;
	ctrl->ops.fpga_release_pinctl = altera_fpga_release_pinctl;
	ctrl->ops.fpga_enable = altera_fpga_enable;
	ctrl->ops.fpga_disable = altera_fpga_disable;
	ctrl->ops.fpga_hold_in_config = altera_fpga_hold_in_config;
	ctrl->ops.fpga_start_config = altera_fpga_start_config;
	ctrl->ops.fpga_poll_config_status = altera_fpga_poll_config_status;
}
