/*
 * Copyright 2018 FLIR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MX7ULP_EC201_CONFIG_H
#define __MX7ULP_EC201_CONFIG_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>


#define CONFIG_BOARD_POSTCLK_INIT
#define CONFIG_SYS_BOOTM_LEN		0x1000000

#define SRC_BASE_ADDR			CMC1_RBASE
#define IRAM_BASE_ADDR			OCRAM_0_BASE
#define IOMUXC_BASE_ADDR		IOMUXC1_RBASE

/* Fuses */
#define CONFIG_CMD_FUSE

#define CONFIG_BOUNCE_BUFFER

/* MMC */
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */
#define CONFIG_SYS_FSL_USDHC_NUM        1
#define CONFIG_SYS_FSL_ESDHC_ADDR       0

/* Using ULP WDOG for reset */
#define WDOG_BASE_ADDR			WDG1_RBASE

#define CONFIG_SYS_ARCH_TIMER
#define CONFIG_SYS_HZ_CLOCK		1000000 /* Fixed at 1Mhz from TSTMR */

#define CONFIG_INITRD_TAG
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
/*#define CONFIG_REVISION_TAG*/

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(8 * SZ_1M)

/* UART */
#define LPUART_BASE			LPUART4_RBASE

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#undef CONFIG_CMD_IMLS

#define CONFIG_SYS_CACHELINE_SIZE      64

/* Miscellaneous configurable options */
#define CONFIG_SYS_PROMPT		"=> "
#define CONFIG_SYS_CBSIZE		512

/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS		256
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_STACKSIZE		SZ_8K

/* Address where u-boot will be loaded in memory */
#define CONFIG_SYS_TEXT_BASE		0x67800000

#define PHYS_SDRAM			0x60000000
#define PHYS_SDRAM_SIZE			SZ_512M
#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM

#define CONFIG_LOADADDR             0x60800000

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_END      0x9E000000


#define CONFIG_SYS_HZ			1000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_STANDALONE_LOAD_ADDR  CONFIG_SYS_LOAD_ADDR

#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	SZ_256K

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#define CONFIG_HX8394

#define CONFIG_SPLASH_SOURCE
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE 1024*1024
#endif
#define CONFIG_CFB_CONSOLE_ANSI
#define CONFIG_OF_BOARD_SETUP

/* Override imx standard behavior when loading u-boot from usb */
#ifdef is_boot_from_usb
#undef is_boot_from_usb
#endif

/* MFG version from separate (easily patchable) file */
#include "flir_mfgmode.h"

#if CONFIG_MFG == 1
/* MFG preloaded recovery boot for board production */

#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE SZ_16K

#define CONFIG_BOOTCOMMAND \
	"setenv ethaddr 00:40:7f:21:22:23; " \
	"fuse prog -y 2 7 8000; " \
	"run partition_mmc_flir; run recboot"

#elif CONFIG_MFG == 2
/* preloaded recovery boot */

#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE SZ_16K

#define CONFIG_BOOTCOMMAND \
        "run recboot"

#else /* CONFIG_MFG */

/* #define CONFIG_SYS_MMC_ENV_DEV		0		/1* emmc0 *1/ */
/* #define CONFIG_ENV_SIZE			SZ_16K */
/* #define CONFIG_ENV_OFFSET		(1792 * 1024)	/1* 256kB below 2MiB *1/ */
/* #define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + (128 * 1024)) */
/* #define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE */

#define CONFIG_BOOTCOMMAND \
	"setmac;" \
	"if recoverytrigger; then " \
		"run recoveryboot;" \
	"else " \
		"run mmcbootflir;" \
	"fi;" \
	"echo Fallback to recovery boot!....;" \
	"run recoveryboot;"

#endif /* CONFIG_MFG */


/* protected environment variables (besides ethaddr and serial#) */
#define CONFIG_ENV_FLAGS_LIST_STATIC	\
	"bootargs_once:sr"


/* Pool of randomly generated UUIDs at host machine */
#define RANDOM_UUIDS	\
	"uuid_disk=075e2a9b-6af6-448c-a52a-3a6e69f0afff\0" \
	"part1_uuid=43f1961b-ce4c-4e6c-8f22-2230c5d532bd\0" \
	"part2_uuid=f241b915-4241-47fd-b4de-ab5af832a0f6\0" \
	"part3_uuid=1c606ef5-f1ac-43b9-9bb5-d5c578580b6b\0" \
	"part4_uuid=c7d8648b-76f7-4e2b-b829-e95a83cc7b32\0" \
	"part5_uuid=ebae5694-6e56-497c-83c6-c4455e12d727\0" \
	"part6_uuid=3845c9fc-e581-49f3-999f-86c9bab515ef\0" \
	"part7_uuid=3fcf7bf1-b6fe-419d-9a14-f87950727bc0\0" \
	"part8_uuid=12c08a28-fb40-430a-a5bc-7b4f015b0b3c\0" \
	"part9_uuid=dc83dea8-c467-45dc-84eb-5e913daec17e\0"


#define CONFIG_OVERWRITE_ETHADDR_ONCE
#define CONFIG_DEFAULT_NETWORK_SETTINGS	\
	"wlanaddr=00:04:f3:ff:ff:fb\0" \
	"btaddr=00:04:f3:ff:ff:fc\0" \
	"ipaddr=192.168.42.30\0" \
	"serverip=192.168.42.1\0" \
	"netmask=255.255.0.0\0"


/* Selects device tree*
 * Reads select-device-tree-script from boot partition
 * and  selects which device tree to use based on board article nr
 * update-fdt updates device tree with board article numbers
 */
#define CONFIG_SELECT_FDT_FILE_ENV \
	"fdt_file=""\0" \
	"fdt_file_default=" CONFIG_DEFAULT_FDT_FILE "\0"\
	"select_fdt_script=select-fdt.uscr\0" \
	"update-fdt=" \
			  "if ext4load mmc ${mmcdev}:${mmcpart} ${tempaddr} /boot/update-fdt.uscr; then " \
					"source ${tempaddr}; " \
			  "fi\0 " \
	"selectfdtfile=" \
			"if test -n ${fdt_file};then " \
				  "echo Devicetree file already selected;" \
			"else " \
				  "if ext4load mmc ${mmcdev}:${mmcpart} ${tempaddr} /boot/${select_fdt_script}; then " \
						"source ${tempaddr}; " \
				  "else;" \
						"setenv fdt_file ${fdt_file_default};" \
				  "fi; " \
		   "fi\0" \
	"selectrecfdtfile=" \
		   "if fatload mmc ${mmcdev}:1 ${tempaddr} ${select_fdt_script}; then " \
				  "source ${tempaddr}; " \
		   "else;" \
				  "setenv fdt_file ${fdt_file_default};" \
		   "fi\0"


/* FLIR partition creation*
 *
 * parition1 = recovery     40 MiB
 * parition2 = rootfs1     512 MiB
 * parition3 = rootfs2     512 MiB
 * parition4 = rootfsrw	   128 MiB
 * parition5 = apps        512 MiB
 * parition6 = data        512 MiB
 * parition7 = storage     rest - about 1600 MiB
 */
#define CONFIG_PARTITION_FLIR_ENV \
	"parts_flir=\"uuid_disk=${uuid_disk};" \
		"start=2MiB,name=recovery,size=40MiB,uuid=${part1_uuid};" \
		"name=rootfs1,size=512MiB,uuid=${part2_uuid};" \
		"name=rootfs2,size=512MiB,uuid=${part3_uuid};" \
		"name=rootfsrw,size=128MiB,uuid=${part4_uuid};" \
		"name=apps,size=512MiB,uuid=${part5_uuid};" \
		"name=data,size=512MiB,uuid=${part6_uuid};" \
		"name=storage,size=-,uuid=${part7_uuid};\"\0" \
	"partition_mmc_flir=mmc rescan; " \
		"if mmc dev ${mmcdev} 0; then " \
			  "gpt write mmc ${mmcdev} ${parts_flir}; " \
			  "mmc rescan; " \
		"else " \
			  "if mmc dev ${mmcdev}; then " \
				  "gpt write mmc ${mmcdev} ${parts_flir}; " \
				  "mmc rescan; " \
			  "else; " \
			  "fi; " \
		"fi; \0"


/* FLIR Recovery boot
*
* Boot from preloaded recovery in ram
* Loads kernel, devicetree and ram rootfs from ram
*
* Boot from recovery partition in mmc
* Loads kernel, devicetree and ram rootfs from recovery partition on emmc
*/
#define CONFIG_REC_BOOT_ENV \
	"recargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/ram0 ethaddr=${ethaddr}\0" \
	"loadinitrd=fatload mmc ${mmcdev}:${mmcpart} ${initrd_addr} ${initrd_file}\0" \
	"initrd_addr=0x63800000\0" \
	"initrd_file=uRamdisk.img\0" \
\
	"recboot=echo Booting preloaded recovery...; "\
		"run recargs; " \
		"bootz ${loadaddr} ${initrd_addr} ${fdt_addr}; \0" \
\
	"recoveryboot=echo Booting from mmc recovery;" \
				 "run selectrecfdtfile; run recargs; " \
				 "setenv bootargs_linux ${bootargs}; " \
				 "setenv mmcpart 1; " \
				 "run loadfdt; run loadinitrd; run loadimage; " \
				 "bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0"


/* Select which emmc partition to boot from
 *
* * Boot from emmc partition2 or partition3
* based on system_active variable
*/

#define CONFIG_SELECT_BOOT_PARTITION_ENV \
	"system_active=system1\0" \
	"select_boot=" \
		"if itest.s ${system_active} == system2; " \
		"then " \
			  "setenv mmcpart 3; setenv rootfs root=/dev/mmcblk0p3; " \
		"else setenv mmcpart 2; setenv rootfs root=/dev/mmcblk0p2; " \
		"fi; \0"

/* common mmc boot
* Environment shared between flir mmc boot and legacy mmc boot
*/
#define CONFIG_MMC_BOOT_COMMON_ENV \
	"mmcdev=0\0" \
	"mmcpart=1\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"${rootfs} rootwait rw ethaddr=${ethaddr} " \
		"wlanaddr=${wlanaddr} btaddr=${btaddr} ${bootargs_once} " \
		 "${extra_bootargs} \0" \
\
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadimage4=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} boot/${image}\0" \
	"loadfdt4=ext4load mmc ${mmcdev}:${mmcpart} ${fdt_addr} " \
		"boot/${fdt_file}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"

/* FLIR mmc boot
*
*loads kernel and boots from partition 2 or 3 on emmc
*/
#define CONFIG_MMC_BOOT_FLIR_ENV \
	"hw_start=run startM4;\0" \
	"setup_boot=bootargs_once=init=/sbin/preinit; " \
			"run select_boot;\0" \
\
	"mmcbootflir=echo Booting from mmc (flir)...;" \
			"bootstate;" \
			"run setup_boot;run update-fdt; run selectfdtfile; " \
			"run hw_start;" \
			"run mmcargs;" \
			"if run loadimage4; then " \
						"if run loadfdt4; then " \
							  "bootz ${loadaddr} - ${fdt_addr}; " \
						"else echo WARN: Cannot load the DT; " \
						"fi; " \
			 "else echo ERR: Cannot load the kernel; " \
			 "fi;\0"

/* Legacy mmc boot
*
* Uses partition layout as created by yocto rootfs.sdcard file
*/
#define CONFIG_MMC_BOOT_LEGACY_ENV \
	"mmcroot_sdcard=/dev/mmcblk0p2  rootwait rw\0" \
	"mmcargs_sdcard=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot_sdcard}\0" \
\
	"mmcboot_sdcard=echo Booting from mmc ...; " \
		"if run loadimage; then " \
			"run mmcargs_sdcard; " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"fi;" \
		"fi;\0" \


/* M4 boot
*
* Loads M4 code from emmc to sram
* Starts executing M4
*/

#define CONFIG_M4_BOOT_ENV \
	"m4_image=imx7ulpm4.bin\0" \
	"m4_addr=0x1ffd2000\0" \
	"m4_bin_size=ext4size mmc ${mmcdev}:${mmcpart} boot/${m4_image}\0" \
	"copyM4=cp.b ${tempaddr} ${m4_addr} ${filesize}\0" \
	"loadM4_to_temp=ext4load mmc ${mmcdev}:${mmcpart} ${tempaddr} boot/${m4_image}\0" \
	"loadM4=if run loadM4_to_temp; then " \
		"run copyM4;" \
	"fi;\0" \
\
	"startM4=echo Starting M4 ...; " \
			"if run loadM4; then " \
				"bootaux ${m4_addr};" \
			"fi;\0"

#define CONFIG_SPLASH_IMAGE_ENV \
	"splashimage=0x67000000\0" \
	"splashsource=mmc_fs\0" \
	"splashfile=/boot/bootlogo.bmp.gz\0" \
	"panel=TRULY-VGA-SHERLOCK\0"

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_DEFAULT_NETWORK_SETTINGS \
	CONFIG_PARTITION_FLIR_ENV \
	RANDOM_UUIDS \
	"image=zImage\0" \
	"console=ttyLP0\0" \
	"tempaddr=0x66000000\0"\
	"fdt_addr=0x63000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"extra_bootargs=quiet\0" \
\
	CONFIG_M4_BOOT_ENV \
	CONFIG_MMC_BOOT_COMMON_ENV\
	CONFIG_SELECT_FDT_FILE_ENV \
	CONFIG_REC_BOOT_ENV \
	CONFIG_SELECT_BOOT_PARTITION_ENV \
	CONFIG_MMC_BOOT_FLIR_ENV\
	CONFIG_MMC_BOOT_LEGACY_ENV \
	CONFIG_SPLASH_IMAGE_ENV \
	""	/* end line */



#endif	/* __MX7ULP_EC201_CONFIG_H */
