/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 FLIR Systems.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __EC501_CONFIG_H
#define __EC501_CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>

#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#ifdef is_boot_from_usb
#undef  is_boot_from_usb
#define is_boot_from_usb() (false)
#endif

#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_ARP_TIMEOUT     200UL

#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

/* Use DA9063 to regulate core voltqages of iMX6 instead of internal LDOs */
#define CONFIG_IMX6_LDO_BYPASS

#define CONFIG_LOADADDR			0x12000000
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN          (10 * 1024 * 1024)
#define CONFIG_SYS_CBSIZE              1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#undef CONFIG_SYS_MAXARGS
#define CONFIG_SYS_MAXARGS             256 // mx6_common
#define CONFIG_SYS_BARGSIZE            CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

/* Physical Memory Map */
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET				\
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR					\
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

// Config main EEPROM here, unless the entire EEPROM driver is used
#define CONFIG_SYS_I2C_EEPROM_BUS      2
#define CONFIG_SYS_I2C_EEPROM_ADDR     0xae
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 1
#define CONFIG_SYS_I2C_SPEED           100000
#define CONFIG_SYS_I2C_MAC_OFFSET // Triggers mac_read_from_eeprom()

#define CONFIG_BOARD_DESCRIPTION       "FLIR ec501 board"
#define CONFIG_MXC_UART_BASE           UART1_BASE
#define CONFIG_CONSOLE_DEV             "ttymxc0"

#define CONFIG_SYS_FSL_USDHC_NUM       1
#define ESDHCI_QUIRK_BROKEN_TIMEOUT_VALUE

#define CONFIG_SYS_USE_SPINOR

/* DA9063 PMIC */
#define DA9063_RW                   0x1 /* Host indicate reading acces via RW=1 */

/* MFG version from separate (easily patchable) file */
#include "ec101_mfgmode.h"

#if CONFIG_MFG == 1
/* MFG preloaded recovery boot for board production */

#define CONFIG_BOOTCOMMAND					\
	"setenv ethaddr 00:40:7f:21:22:23; "			\
	"fuse prog -y 0 6 0x10; fuse prog -y 0 5 0x5860; "	\
	"run partition_mmc_flir; run recboot"

#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE
#ifdef CONFIG_SPLASH_SCREEN
/* No splash screen for MFG - Will destroy preloaded kernel... */
#undef CONFIG_SPLASH_SCREEN
#endif

#elif CONFIG_MFG == 2
/* preloaded recovery boot */

#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE

#define CONFIG_BOOTCOMMAND			\
	"run recboot"

#ifdef CONFIG_SPLASH_SCREEN
/* No splash screen for MFG - Will destroy preloaded kernel... */
#undef CONFIG_SPLASH_SCREEN
#endif

#else
/* Standard ec501 u-boot */

#define CONFIG_BOOTCOMMAND			\
	"if recoverykey && kbd_secret; then "	\
	"run recoveryboot;"			\
	"else "					\
	"chargeState; run mmcbootflir;"		\
	"fi;"					\
	"echo Fallback to recovery boot!....;"	\
	"run recoveryboot;"
#endif /* CONFIG_MFG */

#include "ec501_env.h"

#endif /* #ifndef __EC501_CONFIG_H */

