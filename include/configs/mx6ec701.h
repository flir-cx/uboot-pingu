/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __MX6EC701_CONFIG_H
#define __MX6EC701_CONFIG_H

#include "flir_mx6_common_pre.h"

#define CONFIG_EXTRA_ENV_VARIABLES_SYSTEM  \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	CONFIG_FLIR_DEFAULT_DTB \
	"console=" CONSOLE_DEV "\0" \
	"hw_start=checkCharger; loadFPGA t\0" \
	"" /* EOL */

#define CONFIG_BOARD_DESCRIPTION	"FLIR ec701 board"

#define CONFIG_FLIR_DEFAULT_DTB "fdt_file_default=imx6dl-ec701.dtb\0"
#define CONFIG_EMMC_FUSE_CMD \
	"fuse prog -y 0 6 0x10; fuse prog -y 0 5 0x5860; "
#include "flir_mx6_common_post.h"


/*Spi*/
//#define CONFIG_SYS_USE_SPINOR Temporarily remove to get fpga to be powered
// CONFIG_SF_DEFAULT_CS = IMX_GPIO_NR(5, 28) = 156

#endif                         /* __MX6EC701_CONFIG_H */
