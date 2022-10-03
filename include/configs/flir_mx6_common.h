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

#ifdef CONFIG_NAND_BOOT
#define MFG_NAND_PARTITION "mtdparts=8000000.nor:1m(boot),-(rootfs)\\;gpmi-nand:64m(nandboot),16m(nandkernel),16m(nanddtb),16m(nandtee),-(nandrootfs)"
#else
#define MFG_NAND_PARTITION ""
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

#ifdef CONFIG_SUPPORT_EMMC_BOOT
#define EMMC_ENV \
	"emmcdev=2\0" \
	"update_emmc_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${update_sd_firmware_filename}; then " \
			"if mmc dev ${emmcdev} 1; then "	\
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0"
#else
#define EMMC_ENV ""
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#if defined(CONFIG_NAND_BOOT)
	/*
	 * The dts also enables the WEIN NOR which is mtd0.
	 * So the partions' layout for NAND is:
	 *     mtd1: 16M      (uboot)
	 *     mtd2: 16M      (kernel)
	 *     mtd3: 16M      (dtb)
	 *     mtd4: left     (rootfs)
	 */
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	TEE_ENV \
	"fdt_addr=0x18000000\0" \
	"tee_addr=0x20000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"splashimage=0x28000000\0" \
	"console=" CONSOLE_DEV "\0" \
	"bootargs=console=" CONSOLE_DEV ",115200 ubi.mtd=nandrootfs "  \
		"root=ubi0:nandrootfs rootfstype=ubifs "		     \
		MFG_NAND_PARTITION \
		"\0" \
	"bootcmd=nand read ${loadaddr} 0x4000000 0xc00000;"\
		"nand read ${fdt_addr} 0x5000000 0x100000;"\
		"if test ${tee} = yes; then " \
			"nand read ${tee_addr} 0x4000000 0x400000;"\
			"bootm ${tee_addr} - ${fdt_addr};" \
		"else " \
			"bootz ${loadaddr} - ${fdt_addr};" \
		"fi\0"

#elif defined(CONFIG_SATA_BOOT)

#define CONFIG_EXTRA_ENV_SETTINGS \
		CONFIG_MFG_ENV_SETTINGS \
		TEE_ENV \
		"image=zImage\0" \
		"fdt_file=undefined\0" \
		"fdt_addr=0x18000000\0" \
		"fdt_high=0xffffffff\0"   \
		"splashimage=0x28000000\0" \
		"tee_addr=0x20000000\0" \
		"tee_file=undefined\0" \
		"findfdt="\
			"if test $fdt_file = undefined; then " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6Q; then " \
					"setenv fdt_file imx6q-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-sabreauto.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabresd.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6Q; then " \
					"setenv fdt_file imx6q-sabresd.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-sabresd.dtb; fi; " \
				"if test $fdt_file = undefined; then " \
					"echo WARNING: Could not determine dtb to use; " \
				"fi; " \
			"fi;\0" \
		"findtee="\
			"if test $tee_file = undefined; then " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6QP; then " \
					"setenv tee_file uTee-6qpauto; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6Q; then " \
					"setenv tee_file uTee-6qauto; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6DL; then " \
					"setenv tee_file uTee-6dlauto; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6QP; then " \
					"setenv tee_file uTee-6qpsdb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6Q; then " \
					"setenv tee_file uTee-6qsdb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6DL; then " \
					"setenv tee_file uTee-6dlsdb; fi; " \
				"if test $tee_file = undefined; then " \
					"echo WARNING: Could not determine tee to use; fi; " \
			"fi;\0" \
		"bootargs=console=" CONSOLE_DEV ",115200 \0"\
		"bootargs_sata=setenv bootargs ${bootargs} " \
			"root=/dev/sda2 rootwait rw \0" \
		"bootcmd_sata=run bootargs_sata; scsi scan; " \
			"run findfdt; run findtee;" \
			"fatload scsi 0:1 ${loadaddr} ${image}; " \
			"fatload scsi 0:1 ${fdt_addr} ${fdt_file}; " \
			"if test ${tee} = yes; then " \
				"fatload scsi 0:1 ${tee_addr} ${tee_file}; " \
				"bootm ${tee_addr} - ${fdt_addr}; " \
			"else " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"fi \0"\
		"bootcmd=run bootcmd_sata \0"

#else
/*#define CONFIG_ENV_NXP_FORMAT*/
#ifdef CONFIG_ENV_NXP_FORMAT
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	TEE_ENV \
	"epdc_waveform=epdc_splash.bin\0" \
	"script=boot.scr\0" \
	"image=uImage\0" \
	"fdt_file=undefined\0" \
	"fdt_addr=0x18000000\0" \
	"tee_addr=0x20000000\0" \
	"tee_file=undefined\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV "\0" \
	"dfuspi=dfu 0 sf 0:0:10000000:0\0" \
	"dfu_alt_info_spl=spl raw 0x400\0" \
	"dfu_alt_info_img=u-boot raw 0x10000\0" \
	"dfu_alt_info=spl raw 0x400\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"splashimage=0x28000000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=2\0" \
	"finduuid=part uuid mmc ${mmcdev}:2 uuid\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	EMMC_ENV	  \
	"smp=" SYS_NOSMP "\0"\
	"loadcmd=ext4load\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"${loadcmd} mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script} || " \
		"${loadcmd} mmc ${mmcdev}:${mmcpart} ${loadaddr} boot/${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=${loadcmd} mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image} || " \
		"${loadcmd} mmc ${mmcdev}:${mmcpart} ${loadaddr} boot/${image}\0" \
	"loadfdt=${loadcmd} mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file} || " \
		"${loadcmd} mmc ${mmcdev}:${mmcpart} ${fdt_addr} boot/${fdt_file}\0" \
	"loadtee=${loadcmd} mmc ${mmcdev}:${mmcpart} ${tee_addr} ${tee_file} || " \
		"${loadcmd} mmc ${mmcdev}:${mmcpart} ${tee_addr} boot/${tee_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${tee} = yes; then " \
			"run loadfdt; run loadtee; bootm ${tee_addr} - ${fdt_addr}; " \
		"else " \
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
				"bootz; " \
			"fi;" \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${tee} = yes; then " \
			"${get_cmd} ${tee_addr} ${tee_file}; " \
			"${get_cmd} ${fdt_addr} ${fdt_file}; " \
			"bootm ${tee_addr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
					"bootz ${loadaddr} - ${fdt_addr}; " \
				"else " \
					"if test ${boot_fdt} = try; then " \
						"bootz; " \
					"else " \
						"echo WARN: Cannot load the DT; " \
					"fi; " \
				"fi; " \
			"else " \
				"bootz; " \
			"fi; " \
		"fi;\0" \
		"findfdt="\
			"if test $fdt_file = undefined; then " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6Q; then " \
					"setenv fdt_file imx6q-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-sabreauto.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabresd.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6Q; then " \
					"setenv fdt_file imx6q-sabresd.dtb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-sabresd.dtb; fi; " \
				"if test $board_name = EVCO && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-evco.dtb; fi; " \
				"if test $fdt_file = undefined; then " \
					"echo WARNING: Could not determine dtb to use; fi; " \
			"fi;\0" \
		"findtee="\
			"if test $tee_file = undefined; then " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6QP; then " \
					"setenv tee_file uTee-6qpauto; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6Q; then " \
					"setenv tee_file uTee-6qauto; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6DL; then " \
					"setenv tee_file uTee-6dlauto; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6QP; then " \
					"setenv tee_file uTee-6qpsdb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6Q; then " \
					"setenv tee_file uTee-6qsdb; fi; " \
				"if test $board_name = SABRESD && test $board_rev = MX6DL; then " \
					"setenv tee_file uTee-6dlsdb; fi; " \
				"if test $tee_file = undefined; then " \
					"echo WARNING: Could not determine tee to use; fi; " \
			"fi;\0" \
		""	/* end line */
#define CONFIG_BOOTCOMMAND \
	"run findfdt;" \
	"run findtee;" \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"env set loadcmd; " \
		"for loadcmd in ext4load fatload load; do " \
			"echo trying loadcmd ${loadcmd}; " \
			"if run loadbootscript; then " \
				"run bootscript; " \
			"else " \
				"if run loadimage; then " \
					"run mmcboot; " \
				"fi; " \
			"fi; " \
		"done;" \
	"fi; " \
	"run netboot;"
#else /*CONFIG_ENV_NXP_FORMAT*/
/* protected environment variables (besides ethaddr and serial#) */
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
#elif CONFIG_FLIR_MFG == 2 /* recboot */
#define CONFIG_BOOTCOMMAND \
	"run recboot"
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
#endif /*CONFIG_ENV_NXP_FORMAT*/
#endif

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

#ifdef CONFIG_MTD_NOR_FLASH
#define CONFIG_SYS_FLASH_BASE           WEIM_ARB_BASE_ADDR
#define CONFIG_SYS_FLASH_SECT_SIZE      (128 * 1024)
#define CONFIG_SYS_MAX_FLASH_BANKS 1    /* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT 256   /* max number of sectors on one chip */
#define CONFIG_SYS_FLASH_CFI            /* Flash memory is CFI compliant */
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE /* Use buffered writes*/
#define CONFIG_SYS_FLASH_EMPTY_INFO
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#endif

#ifdef CONFIG_NAND_MXS

#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_NAND_USE_FLASH_BBT

/* DMA stuff, needed for GPMI/MXS NAND support */
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_SPI_BUS             CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS              CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE            CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ          CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_FLASH)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_SYS_SATA_ENV_DEV		0
#endif

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
#if 0
/* PMIC */
#ifndef CONFIG_DM_PMIC
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08
#endif
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

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6sabreandroid_common.h"
#else
#define CONFIG_USBD_HS

#endif /* CONFIG_ANDROID_SUPPORT */
#endif                         /* __MX6EC101_COMMON_CONFIG_H */
