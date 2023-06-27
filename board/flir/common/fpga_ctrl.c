// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include "fpga_ctrl.h"

#include <common.h>
#include <log.h>

int fpga_zero_ctrl(struct fpga_ctrl *fpga)
{
	fpga->pins.ce = -1;
	fpga->pins.config_n = -1;
	fpga->pins.done = -1;
	fpga->pins.init_n = -1;
	fpga->pins.program_n = -1;

	memset(&fpga->board_ops, 0, sizeof(struct fpga_board_ops));
	memset(&fpga->ops, 0, sizeof(struct fpga_ops));
	return 0;
}

int fpga_request_pinctl(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_request_pinctl)
		return fpga->ops.fpga_request_pinctl(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_release_pinctl(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_release_pinctl)
		return fpga->ops.fpga_release_pinctl(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_request_flash_spi(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->board_ops.fpga_request_flash_spi)
		return fpga->board_ops.fpga_request_flash_spi(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_release_flash_spi(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->board_ops.fpga_release_flash_spi)
		return fpga->board_ops.fpga_release_flash_spi(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_init_spi_flash(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->board_ops.fpga_init_spi_flash)
		return fpga->board_ops.fpga_init_spi_flash(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_enable(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_enable)
		return fpga->ops.fpga_enable(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_disable(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_disable)
		return fpga->ops.fpga_disable(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_hold_in_config(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_hold_in_config)
		return fpga->ops.fpga_hold_in_config(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_start_config(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_start_config)
		return fpga->ops.fpga_start_config(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_enable_power(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->board_ops.fpga_enable_power)
		return fpga->board_ops.fpga_enable_power(fpga);
	debug("%s not defined for this FPGA\n", __func__);
	return 0;
}

int fpga_poll_config_status(struct fpga_ctrl *fpga)
{
	debug("Enter %s\n", __func__);
	if (fpga && fpga->ops.fpga_poll_config_status)
		return fpga->ops.fpga_poll_config_status(fpga);
	log_err("%s not defined for this FPGA\n", __func__);
	return -1;
}
