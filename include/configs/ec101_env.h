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


#ifndef __EC101_ENV_H
#define __EC101_ENV_H


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


#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_DEFAULT_NETWORK_SETTINGS \
	RANDOM_UUIDS \
	"script=boot.scr\0" \
	"loadscript=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script}\0" \
	"loadscript4=ext4load mmc ${mmcdev}:${mmcpart} ${loadaddr} " \
                "boot/${script}\0" \
	"uimage=uImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
    "fdt_file_default=imx6dl-evco.dtb\0"      \
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
    "splashimage=0x17000002\0"\
    "tempaddr=0x16000000\0"\
	"fdt_addr=0x18000000\0" \
	"initrd_addr=0x19000000\0" \
	"initrd_file=uRamdisk.img\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
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
#endif

