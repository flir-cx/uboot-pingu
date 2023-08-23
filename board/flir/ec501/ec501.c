// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 FLIR Systems.
 * Copyright (C) 2017 FLIR Systems.
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
#include <init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <i2c.h>
#include <env.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/gpio.h>
#include <asm/mach-imx/boot_mode.h>
#include <linux/fb.h>
#include <i2c.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <fdt_support.h>
#include <version.h>
#include "ec501.h"
#include <spi.h>
#include <asm/arch/mxc_hdmi.h>
#include "../common/flir_generic.h"
#include "../common/da9063.h"
#include "../common/da9063_regs.h"
#include "../common/eeprom.h"
#include "../common/cmd_updatefdteeprom.h"
#include "../common/cmd_loadfpga.h"
#include "../common/board_support.h"
#include "ec501_fpga.h"

DECLARE_GLOBAL_DATA_PTR;
char *get_last_reset_cause(void);
void mxc_mipi_dsi_enable(void);

void imx_bypass_ldo(void);

struct spi_slave *slave; // Extern

int setup_pmic_voltages(void);
int fpga_power(bool enable);
int eth_power(bool enable);
//static void setup_display(void);

#define DA9063_SPI_CS	   0
#define DA9063_SPI_BUS	   3
#define DA9063_SPI_CS_GPIO IMX_GPIO_NR(3, 20)

iomux_v3_cfg_t const ecspi4_pads[] = {
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D20__GPIO3_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL), // <- this sets SION bit,
									  //SION Forces input path of pad ENET_REF_CLK...
	MX6_PAD_ENET_RX_ER__GPIO1_IO24     | MUX_PAD_CTRL(NO_PAD_CTRL),   //Interrupt
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25    | MUX_PAD_CTRL(NO_PAD_CTRL),   //GPIO Reset, Wrong pad name on schematic
	MX6_PAD_ENET_MDC__ENET_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#ifdef CONFIG_SYS_I2C_MXC

/* i2c4: hdmi*/
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_7__I2C4_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 7)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_8__I2C4_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 8)
	}
};

/*i2c3: eeprom, irdm i2c,  */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO_6__GPIO1_IO06 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 6)
	}
};
#endif /* CONFIG_SYS_I2C_MXC */

/* DA9063 Voltages */

/* regulator  | DUPLO	      | Ninjago	     */
/* VBUCKCORE1 | VCCARM	      | VCCARM	     */
/* VBUCKCORE2 | VCCSOC	      | VCCSOC	     */
/* VBUCKIO    | +1V8D	      | +1V8D	     */
/* VBUCKMEM   | 1V1D_FPGA     | 1V1D_FPGA2   */
/* CORE_SW_S  | 1V8D_FPGA     | 1V8D_FPGA    */
/* CORE_SW_G  | SwitchFetGate | SwitchFetgate*/
/* PERI_SWS   | +1V2D_FPGA    | 1V2D_FPGA    */
/* PERI_SWG   | SwitchFetGate | SwitchFetGate*/
/* VBUCKPRO   | 1V0A_ETH      | 1V0D_FPGA    */
/* VDD_CORE   | PMIC_VDDCORE  | PMIC_VDDCORE */
/* VBAT	      | BAT+	      | BAT	     */
/* LDO1	      | NOT USED      | TP23 Notused */
/* LDO2	      | NOT USED      | 1V2D_FPGA2   */
/* LDO3	      | NOT USED      | 1V8D_FPGA2   */
/* LDO4	      | NOT USED      | SD1_VCC	     */
/* LDO5	      | NOT USED      | TP510 Not use*/
/* LDO6	      | 2V5A_ETH      | 2V5D_FPGA2   */
/* LDO7	      | NOT USED      | 3V15D_FPGA2  */
/* LDO8	      | 3V15D_FPGA    | 3V15D_FPGA   */
/* LDO9	      | VCCSNVS	      | VCCSNVS	     */
/* LDO10      | 2V5D_FPGA     | 2V5D_FPGA    */
/* LDO11      | +3V15D	      | +3V15D	     */

//default hw support
/*
  static struct hw_support hardware =
  {
  .mipi_mux =	false,
  .display =	true,
  .usb_charge = false,
  .name = "Unknown Camera"
  };
*/

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	gpio_direction_output(IMX_GPIO_NR(1, 25), 0);
	udelay(1000);
	gpio_direction_output(IMX_GPIO_NR(1, 25), 1);
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
}

#endif /* CONFIG_LDO_BYPASS_CHECK */

#ifdef CONFIG_FSL_ESDHC

/* The order of MMC controllers here must match that of CONFIG_MMCDEV_USDHCx
 * in the platform header
 */

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(struct bd_info *bis)
{
	int i;

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			/* USDHC4 (eMMC) */
			imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[i].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) than supported by the board\n",
			       i + 1);
			return 0;
		}

		if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
			printf("Warning: failed to initialize mmc dev %d\n", i);
	}

	return 0;
}
#endif /* CONFIG_FSL_ESDHC */

#if defined(CONFIG_PHY_TI)
int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif /* defined(CONFIG_PHY_TI) */

/*
 * Override for the weak definition in mxc_spi
 * Note: bus is 0-indexed, in schematic it is 1-indexed
 */
int board_spi_cs_gpio(unsigned int bus, unsigned int cs)
{
	debug("%s(%u, %u)\n", __func__, bus, cs);

	switch (bus) {
	case 0:
		if (cs == 1)
			return IMX_GPIO_NR(5, 28); // FPGA
		break;
	case 1:
		if (cs == 0)
			return IMX_GPIO_NR(2, 26);
		if (cs == 1)
			return IMX_GPIO_NR(2, 27);
		if (cs == 2)
			return IMX_GPIO_NR(2, 28);
	case 3:
		if (cs == 0)
			return IMX_GPIO_NR(3, 20); // DA9063
	}

	return -1;
}

int board_eth_init(struct bd_info *bis)
{
	int ret;

	mac_read_from_eeprom();

	//set default mac address
	if (!env_get("ethaddr")) {
		printf("Setting default ethaddr to 00:04:f3:ff:ff:fa\n");
		env_set("ethaddr", "00:04:f3:ff:ff:fa");
	}
	setup_iomux_enet();
	ret = cpu_eth_init(bis);
	if (ret)
		printf("%s: cpu_eth_init failed\n", __func__);

	return 0;
}

// Initialize boot timer
void board_setup_timer(void)
{
	struct epit *epit_regs = (struct epit *)EPIT1_BASE_ADDR;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	clrbits_le32(&epit_regs->cr, 0x00000001);   // Disable
	setbits_le32(&epit_regs->cr, 0x012C0412);   // 1 MHz free running no output
	setbits_le32(&epit_regs->cr, 0x00000001);   // Enable

	reg = readl(&mxc_ccm->CCGR1);
	reg |= MXC_CCM_CCGR1_EPIT1S_MASK;
	writel(reg, &mxc_ccm->CCGR1);
}

int board_early_init_f(void)
{
	board_setup_timer();
	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	if (hardware.display)
		setup_display();
#endif
	return 0;
}

int board_init(void)
{
	int ret = 0;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C
	/* Setup I2C4  */
	ret = setup_i2c(3, CONFIG_SYS_I2C_SPEED,
			CONFIG_SYS_I2C_SLAVE, &i2c_pad_info1);
	/* Setup I2C3 */
	ret = setup_i2c(2, CONFIG_SYS_I2C_SPEED,
			CONFIG_SYS_I2C_SLAVE, &i2c_pad_info2);
#endif /* CONFIG_SYS_I2C */

	ret = setup_pmic_voltages();
	if (ret)
		return ret;

	ret = eth_power(true);
	if (ret)
		return ret;

#ifdef FLIR_BOARD_SUPPORT
	struct eeprom ioboard = {
		.i2c_bus = 2,
		.i2c_address = 0xaa,
		.i2c_offset = 0x0,
	};
	ret = board_support_setup(&ioboard, &hardware);
#endif

	return ret;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{"emmc2", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif /* CONFIG_CMD_BMODE */

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	setup_spinor();

	return 0;
}

#if defined(CONFIG_OF_BOARD_SETUP)

/* Platform function to modify the FDT as needed */
int ft_board_setup(void *blob, struct bd_info *bd)
{
	uchar enetaddr[6];
	//fix ethernet mac-address using direct path to node
	eth_env_get_enetaddr("ethaddr", enetaddr);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000",
			 "mac-address", &enetaddr, 6, 1);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000",
			 "local-mac-address", &enetaddr, 6, 1);

	do_fixup_by_path_string(blob, "/u-boot", "version", U_BOOT_VERSION_STRING);
	do_fixup_by_path_string(blob, "/u-boot", "reset-cause", get_last_reset_cause());

#if defined(CONFIG_CMD_UPDATE_FDT_EEPROM)
	patch_fdt_eeprom(blob);
#endif
	return 0;
}

#ifndef CONFIG_MXC_SPI
#error "MXC_SPI must be configured"
#endif /* CONFIG_MXC_SPI */
int setup_pmic_voltages(void)
{
	unsigned char dev_id, var_id, cust_id, conf_id;
	struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int ret;

	ret = gpio_request(DA9063_SPI_CS_GPIO, "spi4-cs0");
	if (ret) {
		log_err("Failed to request PMIC CS\n");
		return -1;
	}
	gpio_direction_output(DA9063_SPI_CS_GPIO, 1);
	gpio_free(DA9063_SPI_CS_GPIO);

	imx_iomux_v3_setup_multiple_pads(ecspi4_pads,
					 ARRAY_SIZE(ecspi4_pads));
	// enable ecspi4_clk
	setbits_le32(&ccm_regs->CCGR1, MXC_CCM_CCGR1_ECSPI4S_MASK);
	slave = spi_setup_slave(DA9063_SPI_BUS, DA9063_SPI_CS, 1000000, SPI_MODE_0);
	if (!slave)
		return -1;
	ret = spi_claim_bus(slave);
	if (ret) {
		printf("Could not claim spi bus\n");
		return ret;
	}

	/* Read and print PMIC identification */
	if (pmic_read_reg(DA9063_REG_CHIP_ID, &dev_id) ||
	    pmic_read_reg(DA9063_REG_CHIP_VARIANT, &var_id) ||
	    pmic_read_reg(DA9063_REG_CHIP_CUSTOMER, &cust_id) ||
	    pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id)) {
		printf("Could not read PMIC ID registers\n");
		spi_release_bus(slave);
		return -1;
	}
	printf("PMIC:  DA9063, Device: 0x%02x, Variant: 0x%02x, Customer: 0x%02x, Config: 0x%02x\n",
	       dev_id, var_id, cust_id, conf_id);

	//turn on nONKEY_PIN to port mode, e.g. power switch reacts
	//to button press, instead of button release!
	pmic_write_bitfield(DA9063_REG_CONFIG_I, DA9063_NONKEY_PIN_MASK, DA9063_NONKEY_PIN_PORT);

	//disable comparator
	pmic_write_bitfield(DA9063_REG_ADC_CONT, DA9063_COMP1V2_EN, 0);
	//disable watchdog
	pmic_write_bitfield(DA9063_REG_CONTROL_D, DA9063_TWDSCALE_MASK, 0);

#if defined(CONFIG_IMX6_LDO_BYPASS)
	/* 1V3 is highest allowable voltage when LDO is bypassed */
	if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x64) ||
	    pmic_write_reg(DA9063_REG_VBCORE1_B, 0x64))
		printf("Could not configure VBCORE1 voltage to 1V3\n");
	if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x64) ||
	    pmic_write_reg(DA9063_REG_VBCORE2_B, 0x64))
		printf("Could not configure VBCORE2 voltage to 1V3\n");
	imx_bypass_ldo();
	/* 1V2 is an acceptable level up to 800 MHz */
	if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x5A) ||
	    pmic_write_reg(DA9063_REG_VBCORE1_B, 0x5A))
		printf("Could not configure VBCORE1 voltage to 1V2\n");
	if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x5A) ||
	    pmic_write_reg(DA9063_REG_VBCORE2_B, 0x5A))
		printf("Could not configure VBCORE2 voltage to 1V2\n");
#endif

	spi_release_bus(slave);
	return 0;
}

int eth_power(bool enable)
{
	//Duplo LDO6, VBUCKPRO
	int ret;

	ret = spi_claim_bus(slave);
	if (ret) {
		printf("%s: Failed to claim spi bus\n", __func__);
		return ret;
	}

	// BPRO_EN (1V0_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_BPRO_CONT,
				  DA9063_BUCK_EN,
				  enable ? DA9063_BUCK_EN : 0);
	if (ret) {
		printf("Failed to enable 1V0_FPGA\n");
		return ret;
	}

	// LDO6_EN	    (2V5_ETH)
	ret = pmic_write_bitfield(DA9063_REG_LDO6_CONT,
				  DA9063_LDO_EN,
				  enable ? DA9063_LDO_EN : 0);
	if (ret) {
		printf("Failed to enable 2V5_ETH\n");
		return ret;
	}
	spi_release_bus(slave);

	// DP83867 stabilization time
	// (nominal 200ms, n.b.)
	mdelay(100);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

/*
  struct display_info_t const displays[] = {{
  .bus	= -1,
  .addr	= 0,
  .pixfmt	= IPU_PIX_FMT_RGB24,
  .detect	= NULL,
  .enable	= do_enable_hdmi,
  .mode	= {
  .name		  = "HDMI",
  .refresh	  = 60,
  .xres		  = 640,
  .yres		  = 480,
  .pixclock	  = 39721,
  .left_margin	  = 48,
  .right_margin	  = 16,
  .upper_margin	  = 33,
  .lower_margin	  = 10,
  .hsync_len	  = 96,
  .vsync_len	  = 2,
  .sync		  = 0,
  .vmode	  = FB_VMODE_NONINTERLACED
  } } };
*/
//size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=	MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
		| (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
		| IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
		| IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
		| IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
		| IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
		| IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
		| IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
		| IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
		| IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
		       | IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
		| (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
		   << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#endif /* CONFIG_OF_BOARD_SETUP */
