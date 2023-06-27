// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include "fpga_ctrl.h"

#include <common.h>
#include <asm/gpio.h>
#include <log.h>
#include <linux/delay.h>

static int xilinx_fpga_request_pinctl(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret += gpio_request(fpga->pins.program_n, "fpga-program-n");
	ret += gpio_request(fpga->pins.init_n, "fpga-init-n");
	ret += gpio_request(fpga->pins.done, "fpga-conf-done");

	if (ret) {
		log_err("%s: gpio request failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int xilinx_fpga_release_pinctl(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret += gpio_free(fpga->pins.program_n);
	ret += gpio_free(fpga->pins.init_n);
	ret += gpio_free(fpga->pins.done);

	if (ret) {
		log_err("%s: gpio free failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int xilinx_fpga_disable(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	ret += gpio_direction_output(fpga->pins.program_n, 0);
	ret += gpio_direction_output(fpga->pins.init_n, 0);
	ret += gpio_direction_input(fpga->pins.done);

	if (ret) {
		log_err("%s: gpio set direction failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int xilinx_fpga_hold_in_config(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);

	ret += gpio_direction_output(fpga->pins.init_n, 0);
	ret += gpio_direction_output(fpga->pins.program_n, 0);
	ret += gpio_direction_input(fpga->pins.done);

	if (ret) {
		log_err("%s: gpio set direction failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int xilinx_fpga_start_config(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);

	ret += gpio_direction_output(fpga->pins.init_n, 0);
	ret += gpio_direction_output(fpga->pins.program_n, 0);
	mdelay(10);
	ret += gpio_direction_output(fpga->pins.program_n, 1);
	udelay(10);
	ret += gpio_direction_input(fpga->pins.init_n);

	if (ret) {
		log_err("%s: gpio set direction failed\n", __func__);
		return -ENXIO;
	}
	return 0;
}

static int xilinx_fpga_poll_config_status(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s:\n", __func__);
	// Check if configuration is done
	ret = gpio_get_value(fpga->pins.done);
	debug("%s: done = %d\n", __func__, ret);
	if (ret == 1)
		return 1;

	// Only check for error if done is 0, since init_n is released from fpga
	// and pulled high after successful init
	ret = gpio_get_value(fpga->pins.init_n);
	debug("%s: init_n = %d\n", __func__, ret);
	if (ret == 0)
		return -1;
	return 0;
}

void fpga_set_ops(struct fpga_ctrl *ctrl)
{
	debug("%s: Xilinx fpga chosen\n", __func__);
	ctrl->ops.fpga_request_pinctl = xilinx_fpga_request_pinctl;
	ctrl->ops.fpga_release_pinctl = xilinx_fpga_release_pinctl;
	//ctrl->ops.fpga_enable = xilinx_fpga_enable;
	ctrl->ops.fpga_disable = xilinx_fpga_disable;
	ctrl->ops.fpga_hold_in_config = xilinx_fpga_hold_in_config;
	ctrl->ops.fpga_start_config = xilinx_fpga_start_config;
	ctrl->ops.fpga_poll_config_status = xilinx_fpga_poll_config_status;
}
