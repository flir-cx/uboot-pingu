/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __FLIR_MX6_COMMON_CONFIG_H
#define __FLIR_MX6_COMMON_CONFIG_H

#include <linux/stringify.h>
#include "mx6_common.h"
#include "imx_env.h"
#include "flir_mfgmode.h"
#include "flir_mx6_common_new.h"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_FEC_MXC
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"eth0"

#define CONFIG_PHY_ATHEROS

#ifdef CONFIG_MX6S
#define SYS_NOSMP "nosmp"
#else
#define SYS_NOSMP
#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x12C00000\0" \
	"initrd_high=0xffffffff\0" \
	"emmc_dev=3\0"\
	"sd_dev=2\0" \
	"weim_uboot=0x08001000\0"\
	"weim_base=0x08000000\0"\
	"spi_bus=0\0"\
	"spi_uboot=0x400\0" \
	"mtdparts=" MFG_NAND_PARTITION \
	"\0"\

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_ENV_FLAGS_LIST_STATIC	\
	"wlanaddr:mc,"			\
	"btaddr:mc,"			\
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

/* #define CONFIG_EMMC_FUSE_CMD Should be defined in the file including this one e.g. eoco.h */
#ifndef CONFIG_FLIR_MFG
#error "CONFIG_FLIR_MFG not defined!"
#endif

#if CONFIG_FLIR_MFG == 0 /* Normal boot */
#define CONFIG_BOOTCOMMAND \
	"if recoverykey && kbd_secret; then " \
                "run recoveryboot;" \
	"else " \
                "chargeState; run mmcbootflir;" \
	"fi;" \
	"echo Fallback to recovery boot!....;" \
	"run recoveryboot;"
#elif CONFIG_FLIR_MFG == 1 /* fuse and setup the partitions then recboot */
#define CONFIG_BOOTCOMMAND \
	"setenv ethaddr 00:40:7f:21:22:23; " \
	CONFIG_EMMC_FUSE_CMD \
	"run partition_mmc_flir; run recboot"
#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE
#ifdef CONFIG_SPLASH_SCREEN
/* No splash screen for MFG - Will destroy preloaded kernel... */
#undef CONFIG_SPLASH_SCREEN
#endif
#elif CONFIG_FLIR_MFG == 2 /* recboot */
#define CONFIG_BOOTCOMMAND \
	"run recboot"
#ifdef CONFIG_ENV_IS_IN_MMC
#undef CONFIG_ENV_IS_IN_MMC
#endif
#define CONFIG_ENV_IS_NOWHERE
#else
#error "CONFIG_FLIR_MFG is not set to a valid value!"
#endif /* CONFIG_FLIR_MFG */

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_DEFAULT_NETWORK_SETTINGS \
	RANDOM_UUIDS \
	"script=boot.scr\0" \
	"loadscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script}\0" \
	"loadscript4=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} " \
                "boot/${script}\0" \
	"uimage=uImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	CONFIG_FLIR_DEFAULT_DTB      \
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
           "fi\0" \
    "bootlogo=bootlogo.bmp.gz\0"\
    "loadsplash=run select_boot; ext4load mmc ${mmcdev}:${mmcpart} ${tempaddr} /boot/${bootlogo};cp.w ${tempaddr} ${splashimage} ${filesize}\0" \
    "splashfile=/boot/bootlogo.bmp.gz\0"\
    "splashimage=0x17000002\0"\
    "splashsource=mmc_fs\0"\
    "tempaddr=0x16000000\0"\
	"fdt_addr=0x18000000\0" \
	"initrd_addr=0x19000000\0" \
	"initrd_file=uRamdisk.img\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"hw_start=checkCharger; loadFPGA t\0" \
	"mmcpart=1\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"${rootfs} rootwait rw ethaddr=${ethaddr} " \
                "wlanaddr=${wlanaddr} btaddr=${btaddr} ${bootargs_once} " \
                "${extra_bootargs} ${charge_state}\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loaduimage4=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} boot/${uimage}\0" \
	"loadinitrd=fatload mmc ${mmcdev}:${mmcpart} ${initrd_addr} ${initrd_file}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"loadfdt4=ext4load mmc ${mmcdev}:${mmcpart} ${fdt_addr} " \
                 "boot/${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if run loaduimage; then " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if run loadfdt; then " \
					"bootm ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"if test ${boot_fdt} = try; then " \
						"bootm; " \
					"else " \
						"echo WARN: Cannot load the DT; " \
					"fi; " \
				"fi; " \
			"else " \
				"bootm; " \
			"fi;" \
		"else " \
			"echo ERR: Cannot load the kernel; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${rootpath},v3,tcp " \
        "${extra_bootargs} ${charge_state}\0"\
        "extra_bootargs=quiet\0" \
	"netboot=echo Booting from net ...; " \
                "run hw_start; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${uimage}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file_default}; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
        "recargs=setenv bootargs console=${console},${baudrate} " \
                "root=/dev/ram0 ethaddr=${ethaddr}\0"                \
        "recboot=echo Booting preloaded recovery...; "\
                "run recargs; " \
                "run hw_start;" \
                "bootm ${loadaddr} ${initrd_addr} ${fdt_addr}; \0" \
        "parts_flir=\"uuid_disk=${uuid_disk};" \
                "start=2MiB,name=recovery,size=40MiB,uuid=${part1_uuid};" \
                "name=rootfs1,size=504MiB,uuid=${part2_uuid};" \
                "name=rootfs2,size=504MiB,uuid=${part3_uuid};" \
                "name=rootfsrw,size=128MiB,uuid=${part4_uuid};" \
                "name=apps,size=512MiB,uuid=${part5_uuid};" \
                "name=data,size=512MiB,uuid=${part6_uuid};" \
                "name=spare,size=-,uuid=${part7_uuid};\"\0" \
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
                "fi; \0" \
        "system_active=system1\0" \
        "setup_boot=bootargs_once=init=/sbin/preinit; " \
                "run select_boot;\0" \
        "select_boot=" \
            "if itest.s ${system_active} == system2; " \
            "then " \
                  "setenv mmcpart 3; setenv rootfs root=/dev/mmcblk0p3; " \
            "else setenv mmcpart 2; setenv rootfs root=/dev/mmcblk0p2; " \
            "fi; \0" \
        "mmcbootflir=echo Booting from mmc (flir)...;" \
                "run setup_boot;run update-fdt; run selectfdtfile; " \
                "run hw_start;" \
                "run mmcargs;" \
                "if run loaduimage4; then " \
                      "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                            "if run loadfdt4; then " \
                                  "bootm ${loadaddr} - ${fdt_addr}; " \
                            "else " \
                                  "if test ${boot_fdt} = try; then " \
                                        "bootm;" \
                                  "else echo WARN: Cannot load the DT; " \
                                  "fi; " \
                            "fi; " \
                       "else bootm; " \
                       "fi;" \
                 "else echo ERR: Cannot load the kernel; " \
                 "fi;\0" \
    "recoveryboot=echo Booting from mmc recovery;" \
                 "run selectrecfdtfile; run recargs; " \
                 "run hw_start;" \
                 "setenv bootargs_linux ${bootargs}; " \
                 "setenv mmcpart 1; " \
                 "run loadfdt; run loadinitrd; run loaduimage; " \
                 "bootm ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \
	"mmcroot=PARTUUID=1c606ef5-f1ac-43b9-9bb5-d5c578580b6b\0" \
	"bootargs_mmc_linux=setenv bootargs console=${console},${baudrate} " \
		"${bootargs_linux} root=${mmcroot} rootwait rw " \
		"${bootargs_once} ${extra_bootargs}\0" \
	"bootargs_tftp_linux=setenv bootargs console=${console},${baudrate} " \
		"${bootargs_linux} root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${rootpath},v3,tcp " \
		"${bootargs_once} ${extra_bootargs}\0" \
	"bootargs_nfs_linux=run bootargs_tftp_linux\0" \
	"parts_linux=\"uuid_disk=${uuid_disk};" \
		"start=2MiB," \
		"name=linux,size=64MiB,uuid=${part1_uuid};" \
		"name=linux2,size=64MiB,uuid=${part2_uuid};" \
		"name=rootfs,size=1GiB,uuid=${part3_uuid};" \
		"name=rootfs2,size=1GiB,uuid=${part4_uuid};" \
		"name=userfs,size=-,uuid=${part5_uuid};" \
		"\"\0" \
	"partition_mmc_linux=mmc rescan;" \
		"if mmc dev ${mmcdev} 0; then " \
			"gpt write mmc ${mmcdev} ${parts_linux};" \
			"mmc rescan;" \
		"else " \
			"if mmc dev ${mmcdev};then " \
				"gpt write mmc ${mmcdev} ${parts_linux};" \
				"mmc rescan;" \
			"else;" \
			"fi;" \
		"fi;\0" \
	"mmcdev=0\0" \
	""	/* end line */

#define CONFIG_ARP_TIMEOUT     200UL

/* Physical Memory Map */
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)


/* I2C Configs */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif

#ifdef CONFIG_CMD_I2C
/* #define CONFIG_SYS_I2C_MXC */
/* #define CONFIG_SYS_I2C_MXC_I2C1		/\* enable I2C bus 1 *\/ */
/* #define CONFIG_SYS_I2C_MXC_I2C2		/\* enable I2C bus 2 *\/ */
/* #define CONFIG_SYS_I2C_MXC_I2C3		/\* enable I2C bus 3 *\/ */
/* #define CONFIG_SYS_I2C_SPEED		  100000 */
#endif

/* Framebuffer */
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* 0014-add-bootlogo-functionality.patch */
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE 1024*1024
#define CONFIG_USBD_HS

#endif                         /* __MX6EC101_COMMON_CONFIG_H */
