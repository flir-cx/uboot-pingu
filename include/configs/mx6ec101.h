/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __MX6EC101_CONFIG_H
#define __MX6EC101_CONFIG_H

#include "flir_mx6_common_new.h"

#define CONFIG_BOARD_DESCRIPTION	"FLIR ec101 board"

#define CONFIG_FLIR_DEFAULT_DTB "fdt_file_default=imx6dl-evco.dtb\0"
#define CONFIG_EMMC_FUSE_CMD \
	"fuse prog -y 0 6 0x10; fuse prog -y 0 5 0x5860; "
#include "flir_mx6_common.h"


/*Spi*/
#define CONFIG_SYS_USE_SPINOR
// CONFIG_SF_DEFAULT_CS = IMX_GPIO_NR(5, 28) = 156

#endif                         /* __MX6EC101_CONFIG_H */
