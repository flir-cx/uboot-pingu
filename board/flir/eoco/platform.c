/*
 * Copyright (C) 2015 FLIR Systems.
 * Copyright (C) 2017 FLIR Systems.
 * Copyright (C) 2022 Teledyne FLIR Systems.
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
#include <common.h>
#include <command.h>
#include <init.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/crm_regs.h>
#include <errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <ipu_pixfmt.h>
#include <linux/fb.h>
#include <i2c.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <linux/string.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <fuse.h>
#include <miiphy.h>
#include <netdev.h>
#include <part.h>
#include <fdt_support.h>
#include <version.h>
#include <pwm.h>
// TODO: add this
//#include <cmd_updatefdteeprom.h>
// TODO: add this
//#include <da9063.h>
#include "../../../flir/include/eeprom.h"
#include <stdio_dev.h>
// TODO: add this
//#include <cmd_loadfpga.h>
// TODO: add this
//#include <cmd_kbd.h>
#include "../../../flir/include/flir_generic.h"
// TODO: add this
//#include <usbcharge.h>
/* #include "ec101.h" */
/* #include "../../../flir/include/board_support.h" */
/* #include "../../../flir/include/usbcharge.h" */
#include <spi.h>
/* #include <linux/delay.h> */
/* #include <asm-generic/u-boot.h> */

#include "../../../flir/include/da9063.h"
#include "../../../flir/include/da9063_regs.h"
#include "../../../flir/include/cmd_loadfpga.h"
/* #include "../../../flir/include/cmd_updatefdteeprom.h" */

/* DECLARE_GLOBAL_DATA_PTR; */
/* char *get_last_reset_cause(void); */
/* void mxc_mipi_dsi_enable(void); */

/* void imx_bypass_ldo(void); */
extern struct spi_slave *slave;
/* int setup_pmic_voltages(void); */
int fpga_power(bool enable);

/* //default hw support */
/* static struct hw_support hardware = */
/* { */
/* 	.mipi_mux =	false, */
/* 	.display = true, */
/* 	.usb_charge = false, */
/* 	.name = "Unknown Camera" */
/* }; */


/* int dram_init(void) */
/* { */
/* 	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024); */
/* 	return 0; */
/* } */

/* static void setup_iomux_enet(void) */
/* { */
/* 	imx_iomux_v3_setup_multiple_pads(enet_pads_100, ARRAY_SIZE(enet_pads_100)); */

/* 	gpio_direction_output(IMX_GPIO_NR(6, 19), 0); */
/* 	udelay(1000); */
/* 	gpio_direction_output(IMX_GPIO_NR(6, 19), 1); */
/* } */

/* static void setup_iomux_uart(void) */
/* { */
/* 	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads)); */
/* } */

/* /\* */
/*  * Do not overwrite the console */
/*  * Use always serial for U-Boot console */
/*  *\/ */
/* int overwrite_console(void) */
/* { */
/* 	return 1; */
/* } */

/* int board_mmc_get_env_dev(int devno) */
/* { */
/* 	return CONFIG_MMCDEV_USDHC4; */
/* } */

/* #if defined(CONFIG_VIDEO_IPUV3) */
/* void splash_screen_prepare(void) */
/* { */
/* 	char *env_loadsplash; */

/* 	set_boot_logo(); */

/* 	if (!env_get("splashimage")) { */
/* 		return; */
/* 	} */

/* 	env_loadsplash = env_get("loadsplash"); */
/* 	if (env_loadsplash == NULL) { */
/* 		printf("Environment variable loadsplash not found!\n"); */
/* 		return; */
/* 	} */

/* 	if (run_command_list(env_loadsplash, -1, 0)) { */
/* 		printf("failed to run loadsplash %s\n\n", env_loadsplash); */
/* 	} */

/* 	return; */
/* } */

/* int board_video_skip(void) */
/* { */
/* 	return 0; */
/* } */

/*  void setup_display(void) */
/* { */
/* 	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR; */
/* 	int reg; */
/* 	unsigned char buf[2]; */
/* 	i2c_set_bus_num(2); */

/* 	/\* Turn on IPU1 and DI0 clocks *\/ */
/* 	reg = readl(&mxc_ccm->CCGR3); */
/* 	reg |= MXC_CCM_CCGR3_IPU1_IPU_MASK | MXC_CCM_CCGR3_IPU1_IPU_DI0_MASK; */
/* 	writel(reg, &mxc_ccm->CCGR3); */

/* 	reg = readl(&mxc_ccm->cscdr3); */
/* 	reg |= MXC_CCM_CSCDR3_IPU1_HSP_CLK_SEL_MASK; */
/* 	reg &= ~MXC_CCM_CSCDR3_IPU1_HSP_PODF_MASK; */
/* 	reg |= (0x1 << MXC_CCM_CSCDR3_IPU1_HSP_PODF_OFFSET); */
/* 	writel(reg, &mxc_ccm->cscdr3); */

/*     //init backlight to lcd */
/*     pwm_init(PWM1, 0, 0); */
/*     //duty cycle = 70%, period = 0.2ms (should match duty cycle in kernel) */
/*     pwm_config(PWM1, 140000, 200000); */
/*     pwm_enable(PWM1); */
/*     imx_iomux_v3_setup_multiple_pads(backlight_pads, */
/* 				     ARRAY_SIZE(backlight_pads)); */

/* 	if(hardware.mipi_mux) */
/* 	{ */
/* 		//set LCD_MIPI_SEL=1 and LCD_MIPI_EN=0 */
/* 		buf[0]= 0xbf; */
/* 		i2c_write(LEIF_PCA9534_ADDRESS, 0x1, 1, buf, 1); */
/* 		buf[0]= 0x9f; */
/* 		i2c_write(LEIF_PCA9534_ADDRESS, 0x3, 1, buf, 1); */
/* 	} */

/* 	mxc_mipi_dsi_enable(); */
/* } */
/* #endif */


/* #ifdef CONFIG_LDO_BYPASS_CHECK */
/* void ldo_mode_set(int ldo_bypass) */
/* { */
/* } */

/* #endif */


/* #ifdef CONFIG_FSL_ESDHC */
/* /\* The order of MMC controllers here must match that of CONFIG_MMCDEV_USDHCx */
/*  * in the platform header */
/*  *\/ */
/* struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = { */
/* 	{ */
/* 		.esdhc_base = USDHC4_BASE_ADDR, */
/* 		.max_bus_width = 8 */
/* 	}, */
/* }; */

/* int board_mmc_getcd(struct mmc *mmc) */
/* { */
/* 	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv; */
/* 	int ret = 0; */

/* 	switch (cfg->esdhc_base) { */
/* 	case USDHC4_BASE_ADDR: */
/* 		ret = 1; /\* eMMC/uSDHC4 is always present *\/ */
/* 		break; */
/* 	} */

/* 	return ret; */
/* } */

/* int board_mmc_init(struct bd_info *bis) */
/* { */
/* 	int i; */

/* 	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) { */
/* 		switch (i) { */
/* 		case 0: */
/* 			/\* USDHC4 (eMMC) *\/ */
/* 			imx_iomux_v3_setup_multiple_pads( */
/* 					usdhc4_pads, ARRAY_SIZE(usdhc4_pads)); */
/* 			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK); */
/* 			break; */
/* 		default: */
/* 			printf("Warning: you configured more USDHC controllers" */
/* 				"(%d) than supported by the board\n", i + 1); */
/* 			return 0; */
/* 		} */

/* 		if (fsl_esdhc_initialize(bis, &usdhc_cfg[i])) */
/* 			printf("Warning: failed to initialize mmc dev %d\n", i); */
/* 	} */

/* 	return 0; */
/* } */
/* #endif */


/* #if defined(CONFIG_PHY_MICREL) */
/* int board_phy_config(struct phy_device *phydev) */
/* { */
/* 	if (phydev->drv->config) */
/* 		phydev->drv->config(phydev); */

/* 	return 0; */
/* } */
/* #endif */

/* int board_eth_init(struct bd_info *bis) */
/* { */
/* #if 0 */
/* 	int ret; */

/* 	mac_read_from_eeprom(); */

/* 	//set default mac address */
/* 	if (!env_get("ethaddr")) { */
/* 		printf("Setting default ethaddr to 00:04:f3:ff:ff:fa \n"); */
/* 		env_set("ethaddr", "00:04:f3:ff:ff:fa"); */
/* 	} */
/* 	setup_iomux_enet(); */
/* 	ret = cpu_eth_init(bis); */
/* 	if (ret) */
/* 		printf("%s: cpu_eth_init failed\n", __func__); */
/* #endif */
/* 	return 0; */
/* } */

/* int board_spi_cs_gpio(unsigned bus, unsigned cs) */
/* { */
/*        return cs; */
/* } */

/* static void setup_buttons(void) */
/* { */
/*     gpio_direction_input(IMX_GPIO_NR(7, 11)); */
/*     imx_iomux_v3_setup_multiple_pads(recovery_btn_pad, ARRAY_SIZE(recovery_btn_pad)); */
/* } */

/* // Initialize boot timer */
/* void board_setup_timer(void) */
/* { */
/* 	struct epit *epit_regs = (struct epit *)EPIT1_BASE_ADDR; */
/* 	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR; */
/* 	int reg; */

/* 	clrbits_le32(&epit_regs->cr, 0x00000001);   // Disable */
/* 	setbits_le32(&epit_regs->cr, 0x012C0412);   // 1 MHz free running no output */
/* 	setbits_le32(&epit_regs->cr, 0x00000001);   // Enable */

/* 	reg = readl(&mxc_ccm->CCGR1); */
/* 	reg |= MXC_CCM_CCGR1_EPIT1S_MASK; */
/* 	writel(reg, &mxc_ccm->CCGR1); */
/* } */


/* int board_early_init_f(void) */
/* { */
/* 	board_setup_timer(); */
/* 	setup_iomux_uart(); */
/* 	setup_buttons(); */

/* 	return 0; */
/* } */

/* int board_init(void) */
/* { */
/* 	int ret = 0; */
/* 	/\* address of boot parameters *\/ */
/* 	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100; */

/* #ifdef CONFIG_SYS_I2C_MXC */
/* 	/\* Setup I2C4  *\/ */
/* 	ret = setup_i2c(3, CONFIG_SYS_I2C_SPEED, */
/* 			CONFIG_SYS_I2C_SLAVE, &i2c_pad_info1); */
/* 	/\* Setup I2C3 *\/ */
/* 	ret = setup_i2c(2, CONFIG_SYS_I2C_SPEED, */
/* 			CONFIG_SYS_I2C_SLAVE, &i2c_pad_info2); */
/* 	ret = setup_pmic_voltages(); */
/* 	if (ret) */
/* 		return -1; */

/* 	struct Eeprom ioboard = */
/* 	{ */
/* 	 .i2c_bus = 2, */
/* 	 .i2c_address = 0xaa, */
/* 	 .i2c_offset = 0x0, */
/* 	 //.article = 0, */
/* 	 //.revision = 0, */
/* 	 //.name = "\0", */
/* 	}; */


/* 	board_support_setup(&ioboard, &hardware); */
/* #endif */

/* 	if(hardware.usb_charge) */
/* 		usb_charge_setup(); */

/* #if defined(CONFIG_VIDEO_IPUV3) */
/* 	if(hardware.display) */
/* 		setup_display(); */
/* #endif */
/* 	return ret; */
/* } */

/* #ifdef CONFIG_CMD_BMODE */
/* static const struct boot_mode board_boot_modes[] = { */
/* 	/\* 8 bit bus width *\/ */
/*     {"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)}, */
/*     {"emmc2", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)}, */
/* 	{NULL,	 0}, */
/* }; */
/* #endif */


/* int board_late_init(void) */
/* { */
/* #ifdef CONFIG_CMD_BMODE */
/* 	add_board_boot_modes(board_boot_modes); */
/* #endif */

/* #ifdef CONFIG_SYS_USE_SPINOR */
/*        setup_spinor(); */
/* #endif */
/* 	return 0; */
/* } */

/* #if defined(CONFIG_OF_BOARD_SETUP) */

/* /\* Platform function to modify the FDT as needed *\/ */
/* int ft_board_setup(void *blob, struct bd_info *bd) */
/* { */
/* 	uchar enetaddr[6]; */
/* 	//fix ethernet mac-address using direct path to node */
/* 	eth_env_get_enetaddr("ethaddr", enetaddr); */
/* 	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000", "mac-address", &enetaddr, 6, 1); */
/* 	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000", "local-mac-address", &enetaddr, 6, 1); */

/* 	do_fixup_by_path_string(blob, "/u-boot", "version", U_BOOT_VERSION_STRING); */
/* 	do_fixup_by_path_string(blob, "/u-boot", "reset-cause", get_last_reset_cause()); */

/* #if defined(CONFIG_VIDEO_IPUV3) */
/*     int temp[2]; */
/*     temp[0] = cpu_to_fdt32(gd->fb_base); */
/*     temp[1] = cpu_to_fdt32(640*480*2); */
/*     do_fixup_by_path(blob, "/fb@0", "bootlogo", temp, sizeof(temp), 0); */
/* #endif */

/* #if defined(CONFIG_CMD_UPDATE_FDT_EEPROM) */
/*     patch_fdt_eeprom(blob); */
/* #endif */
/* 	return 0; */
/* } */


/* int setup_pmic_voltages(void) */
/* { */
/*     unsigned char dev_id, var_id, cust_id, conf_id; */
/*     struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR; */

/*     imx_iomux_v3_setup_multiple_pads(ecspi4_pads, */
/*                                      ARRAY_SIZE(ecspi4_pads)); */
/*     // enable ecspi4_clk */
/*     setbits_le32(&ccm_regs->CCGR1, MXC_CCM_CCGR1_ECSPI4S_MASK); */
/*     slave = spi_setup_slave(DA9063_SPI_BUS, DA9063_SPI_CS, 1000000, SPI_MODE_0); */
/*     if (!slave) */
/*         return -1; */
/*     spi_claim_bus(slave); */

/*     /\* Read and print PMIC identification *\/ */
/*     if (pmic_read_reg(DA9063_REG_CHIP_ID, &dev_id) || */
/*             pmic_read_reg(DA9063_REG_CHIP_VARIANT, &var_id) || */
/*             pmic_read_reg(DA9063_REG_CHIP_CUSTOMER, &cust_id) || */
/*             pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id)) { */
/*         printf("Could not read PMIC ID registers\n"); */
/*         spi_release_bus(slave); */
/*         return -1; */
/*     } */
/*     printf("PMIC:  DA9063, Device: 0x%02x, Variant: 0x%02x, " */
/*            "Customer: 0x%02x, Config: 0x%02x\n", dev_id, var_id, */
/*            cust_id, conf_id); */

/*     //turn on nONKEY_PIN to port mode, e.g. power switch reacts */
/*     //to button press, instead of button release! */
/*     pmic_write_bitfield(DA9063_REG_CONFIG_I, DA9063_NONKEY_PIN_MASK, DA9063_NONKEY_PIN_PORT); */

/* 	//disable comparator */
/* 	pmic_write_bitfield(DA9063_REG_ADC_CONT, DA9063_COMP1V2_EN, 0); */
/* 	//disable watchdog */
/* 	pmic_write_bitfield(DA9063_REG_CONTROL_D, DA9063_TWDSCALE_MASK, 0); */

/* #if defined(CONFIG_IMX6_LDO_BYPASS) */
/*     /\* 1V3 is highest allowable voltage when LDO is bypassed *\/ */
/*     if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x64) || */
/* 	pmic_write_reg(DA9063_REG_VBCORE1_B, 0x64)) */
/* 	    printf("Could not configure VBCORE1 voltage to 1V3\n"); */
/*     if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x64) || */
/* 	pmic_write_reg(DA9063_REG_VBCORE2_B, 0x64)) */
/* 	    printf("Could not configure VBCORE2 voltage to 1V3\n"); */
/*     imx_bypass_ldo(); */
/*     /\* 1V2 is an acceptable level up to 800 MHz *\/ */
/*     if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x5A) || */
/* 	pmic_write_reg(DA9063_REG_VBCORE1_B, 0x5A)) */
/* 	    printf("Could not configure VBCORE1 voltage to 1V2\n"); */
/*     if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x5A) || */
/* 	pmic_write_reg(DA9063_REG_VBCORE2_B, 0x5A)) */
/* 	    printf("Could not configure VBCORE2 voltage to 1V2\n"); */
/* #endif */

/*     spi_release_bus(slave); */
/*     return 0; */
/* } */



int fpga_power(bool enable)
{
    unsigned char conf_id;
    if (pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id)) {
        printf("Could not read PMIC ID registers\n");
        spi_release_bus(slave);
        return -1;
    }

	spi_claim_bus(slave);
    // BPRO_EN (1V0_FPGA)
    pmic_write_bitfield(DA9063_REG_BPRO_CONT,DA9063_BUCK_EN,enable?DA9063_BUCK_EN:0);
    // CORE_SW_EN  (1V8_FPGA)
    pmic_write_bitfield(DA9063_REG_BCORE1_CONT,DA9063_CORE_SW_EN,enable?DA9063_CORE_SW_EN:0);
    // PERI_SW_EN    (1V2_FPGA)
    pmic_write_bitfield(DA9063_REG_BPERI_CONT,DA9063_PERI_SW_EN,enable?DA9063_PERI_SW_EN:0);
	if(conf_id == 0x3b) //revC
	{
		// BMEM_EN         (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_BMEM_CONT,DA9063_BUCK_EN,enable?DA9063_BUCK_EN:0);
		// LDO10_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT,DA9063_LDO_EN,enable?DA9063_LDO_EN:0);
	}
	else //revD
	{
		// LDO10_EN          (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT,DA9063_LDO_EN,enable?DA9063_LDO_EN:0);
		// LDO8_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO8_CONT,DA9063_LDO_EN,enable?DA9063_LDO_EN:0);
	}

	//Enable BMEM_CONT regualtor, 1V1_FPGA, FPGA Core voltage, Only EOCO??
	pmic_write_bitfield(DA9063_REG_BMEM_CONT,DA9063_BUCK_EN,enable?DA9063_BUCK_EN:0);
	spi_release_bus(slave);
	return 0;
	
}

/* #endif /\* CONFIG_OF_BOARD_SETUP *\/ */
