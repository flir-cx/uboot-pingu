/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __EOCO_CONFIG_H
#define __EOCO_CONFIG_H

#include "flir_mx6_common_pre.h"

#define CONFIG_FLIR_DEFAULT_DTB "fdt_file_default=imx6qp-eoco.dtb\0"
#define CONFIG_EMMC_FUSE_CMD "fuse prog -y 0 6 0x00000010; fuse prog -y 0 5 0x00205860;"

#define CONFIG_EXTRA_ENV_VARIABLES_SYSTEM \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	CONFIG_FLIR_DEFAULT_DTB \
	"console=" CONSOLE_DEV "\0" \
	"hw_start=loadFPGA t\0" \
	"" /* EOL */

#include "flir_mx6_common_post.h"

#if CONFIG_FLIR_MFG == 0 /* Normal boot */
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
	"if recoverykey; then run recoveryboot;" \
	"else run mmcbootflir;" \
	"fi;" \
	"echo Fallback to recovery boot!....;" \
	"run recoveryboot;"
#endif

#define CONFIG_IMX6_LDO_BYPASS

/* FUELGAUGE BQ40Z50 */
#define BQ40Z50_I2C_ADDR 0x0b
#define BQ40Z50_I2C_BUS 1
#define BQ40Z50_REG_STATE_OF_CHARGE 0x0d
#define BQ40Z50_BATT_CRITICAL_LEVEL 0x0

#endif                         /* __EOCO_CONFIG_H */
