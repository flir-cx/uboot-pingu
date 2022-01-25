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
#ifndef __EC101_CONFIG_H
#define __EC101_CONFIG_H

//#define CONFIG_MX6
#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
//#include <asm/imx-common/gpio.h>

//#define CONFIG_DISPLAY_CPUINFO
//#define CONFIG_DISPLAY_BOARDINFO_LATE

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

//#define CONFIG_SYS_L2CACHE_OFF

//#define CONFIG_BOARD_EARLY_INIT_F
//#define CONFIG_BOARD_LATE_INIT
//#define CONFIG_MXC_GPIO
//#define CONFIG_MXC_UART

#define CONFIG_SYS_GENERIC_BOARD

#ifdef is_boot_from_usb
#undef  is_boot_from_usb
#define is_boot_from_usb(void) (false)
#endif

/* MMC Configs */
//#define CONFIG_FSL_ESDHC
//#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
/*#define CONFIG_MMC
#define CONFIG_CMD_MMC*/
#define CONFIG_GENERIC_MMC
#define CONFIG_CMD_EMMC
/*#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_FAT
#define CONFIG_FAT_WRITE
#define CONFIG_DOS_PARTITION
#define CONFIG_EFI_PARTITION
#define CONFIG_CMD_GPT
#define CONFIG_PARTITION_UUIDS
#define CONFIG_CMD_PART
*/

/*Ethernet config*/
//#define CONFIG_CMD_PING
//#define CONFIG_CMD_DHCP
//#define CONFIG_CMD_MII
//#define CONFIG_CMD_NET
//#define CONFIG_FEC_MXC
//#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_ETHPRIME			"FEC"
//#define CONFIG_PHYLIB
#define CONFIG_ARP_TIMEOUT     200UL
//#define CONFIG_PHY_MICREL
#define CONFIG_ENET_PHYADDR_MICREL	0
#define CONFIG_FEC_MXC_PHYADDR CONFIG_ENET_PHYADDR_MICREL
#define CONFIG_FEC_XCV_TYPE             RMII

#undef CONFIG_CMD_USB
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


//#define CONFIG_CONS_INDEX              1
//#define CONFIG_BAUDRATE                        115200

/* Command definition */
//#include <config_cmd_default.h>
//#define CONFIG_CMD_BMODE
//#define CONFIG_CMD_BOOTZ
//#undef CONFIG_CMD_IMLS
//#define CONFIG_CMD_SETEXPR
//#define CONFIG_CMD_FUSE
//#ifdef CONFIG_CMD_FUSE
//#define CONFIG_MXC_OCOTP
//#endif

/* Use DA9063 to regulate core voltqages of iMX6 instead of internal LDOs */
#define CONFIG_IMX6_LDO_BYPASS

//#define CONFIG_CMD_GPIO
#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif
//#define CONFIG_SOURCE 
//#define CONFIG_CMD_TIME
#define CONFIG_READMAINBOARDREVISION
#define CONFIG_CMD_UPDATE_FDT_EEPROM

#define CONFIG_CMD_BOARD

#define CONFIG_LOADADDR			0x12000000
//#define CONFIG_SYS_TEXT_BASE		0x17800000
/* RAM memory reserved for U-Boot, stack, malloc pool... */
#define CONFIG_UBOOT_RESERVED		(10 * 1024 * 1024)
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * 1024 * 1024)


/* Miscellaneous configurable options */
//#define CONFIG_SYS_LONGHELP
//#define CONFIG_SYS_HUSH_PARSER
//#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
//#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
//#define CONFIG_SYS_MAXARGS             256
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
//#define CONFIG_CMDLINE_EDITING

//#define CONFIG_SYS_MEMTEST_START       0x10000000
//#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
//#define CONFIG_SYS_HZ                  1000


#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */


/* Physical Memory Map */
//#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
//#define CONFIG_ENV_SIZE		(16 * 1024)
#define CONFIG_SYS_NO_FLASH

//#define CONFIG_OF_LIBFDT
//#define CONFIG_OF_BOARD_SETUP

/* Framebuffer and LCD */
//#define CONFIG_VIDEO
//#define CONFIG_VIDEO_IPUV3
/*#define CONFIG_CFB_CONSOLE
#define CONFIG_CFB_CONSOLE_ANSI
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_BMP_16BPP
#define CONFIG_SPLASHIMAGE_GUARD*/
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE		(1024*1024)
//#define CONFIG_CMD_BMP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK 260000000

//#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK 66000000
/*
 * I2C configs
 */
//#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
//#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_EEPROM_BUS_NUM   2
#define CONFIG_SYS_I2C_EEPROM_ADDR   (0xae>>1)
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 1


#define CONFIG_SYS_BOARD "ec101"
#define CONFIG_BOARD_DESCRIPTION	"FLIR ec101 board"
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"
//#define CONFIG_DEFAULT_FDT_FILE

#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_MMCDEV_USDHC4		0	/* mmc index for SHDC4 (eMMC) */
//#define CONFIG_SYS_MMC_ENV_DEV		CONFIG_MMCDEV_USDHC4

/*Spi*/
#define CONFIG_SYS_USE_SPINOR
#ifdef CONFIG_SYS_USE_SPINOR
//#define CONFIG_CMD_SF
//#define CONFIG_SPI_FLASH
//#define CONFIG_SPI_FLASH_STMICRO
//#define CONFIG_SPI_FLASH_BAR
//#define CONFIG_MXC_SPI
//#define CONFIG_SF_DEFAULT_BUS  0
//#define CONFIG_SF_DEFAULT_SPEED 20000000
//#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
//#define CONFIG_SF_DEFAULT_CS  IMX_GPIO_NR(5, 28)
#endif
//#define CONFIG_CMD_SPI		/* SPI utility			*/
/* DA9063 PMIC */
#define DA9063_RW                   0x1 /* Host indicate reading acces via RW=1 */
#define DA9063_SPI_CS               IMX_GPIO_NR(3, 20)
#define DA9063_SPI_BUS              3



/* Celsius degrees below CPU's max die temp at which boot should be attempted */
#define CONFIG_BOOT_TEMP_BELOW_MAX              10
#define CONFIG_BOOTDELAY 0

/* MFG version from separate (easily patchable) file */
#include "ec101_mfgmode.h"

#if CONFIG_MFG == 1
/* MFG preloaded recovery boot for board production */

#define CONFIG_BOOTCOMMAND \
	"setenv ethaddr 00:40:7f:21:22:23; " \
        "fuse prog -y 0 6 0x10; fuse prog -y 0 5 0x5860; " \
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

#define CONFIG_BOOTCOMMAND \
	"run recboot"

#ifdef CONFIG_SPLASH_SCREEN
/* No splash screen for MFG - Will destroy preloaded kernel... */
#undef CONFIG_SPLASH_SCREEN
#endif

#else
/* Standard ec101 u-boot */

//#define CONFIG_ENV_IS_IN_MMC
//#if defined(CONFIG_ENV_IS_IN_MMC)
//#define CONFIG_ENV_OFFSET		(1792 * 1024)	/* 256kB below 2MiB */
//#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + (128 * 1024))
//#define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE
//#endif

#define CONFIG_BOOTCOMMAND \
	"if recoverykey && kbd_secret; then " \
                "run recoveryboot;" \
	"else " \
                "chargeState; run mmcbootflir;" \
	"fi;" \
	"echo Fallback to recovery boot!....;" \
	"run recoveryboot;"
#endif /* CONFIG_MFG */

#include "ec101_env.h"

#define CONFIG_FPGA_XILINX
#endif
