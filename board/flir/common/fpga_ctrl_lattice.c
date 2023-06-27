// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include "fpga_ctrl.h"

static int lattice_fpga_request_pinctl(struct fpga_ctrl *fpga)
{
	return 0;
}

void fpga_set_ops(struct fpga_ctrl *ctrl)
{
	debug("%s: Lattice fpga chosen\n", __func__);
	ctrl->ops.fpga_request_pinctl = lattice_fpga_request_pinctl;
}

