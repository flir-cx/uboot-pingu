/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 FLIR Systems
 *
 * Configuration settings for the FLIR EC702 board
 */

#ifndef __EC702_CONFIG_H
#define __EC702_CONFIG_H
#define CONFIG_SYS_USE_SPINOR

#include "flir_mx6_common_pre.h"

#define CONFIG_FLIR_DEFAULT_DTB "fdt_file_default=imx6qp-ec702.dtb\0"
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

// Config main EEPROM here, unless the entire EEPROM driver is used
#define CONFIG_SYS_I2C_EEPROM_BUS      0
#define CONFIG_SYS_I2C_EEPROM_ADDR     0xae
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 1

#endif /* __EC702_CONFIG_H */
