// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 FLIR Automation
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/video.h>
#include <env.h>
#include <fdt_support.h>
#include <fsl_esdhc_imx.h>
#include <i2c.h>
#include <image.h>
#include <init.h>
#include <input.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <miiphy.h>
#include <mmc.h>
#include <net.h>
#include <power/regulator.h>
#include <pwm.h>
#include <spi.h>
#include <usb.h>
#include <usb/ehci-ci.h>

#include "../../../drivers/video/mxc_mipi_dsi.h"
#include "../../../drivers/video/mxcfb_st7703.h"
#include "../common/cmd_loadfpga.h"
#include "../common/cmd_updatefdteeprom.h"
#include "../common/da9063_regs.h"
#include "../common/da9063.h"
#include "../common/flir_generic.h"
#include "../common/fpga_ctrl.h"
#include "../common/usbcharge.h"

#include "ec701.h"

DECLARE_GLOBAL_DATA_PTR;

#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

struct spi_slave *slave; // Extern

/*
 * Setup DRAM size. Invoked by init_sequence_f
 */
int dram_init(void)
{
	/* Since imx_ddr_size does not know about interleaved multiply by 2 */
	/* It calculates only on one mem capsule */
	gd->ram_size = imx_ddr_size() * 2;

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};


#ifdef CONFIG_MXC_SPI

static iomux_v3_cfg_t const ecspi4_pads[] = {
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D20__GPIO3_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int platform_setup_pmic_voltages(void)
{
	unsigned char dev_id, var_id, cust_id, conf_id;
	struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	gpio_request(IMX_GPIO_NR(3, 20), "CS SPI4");
	gpio_direction_output(IMX_GPIO_NR(3, 20), 1);

	imx_iomux_v3_setup_multiple_pads(ecspi4_pads,
					 ARRAY_SIZE(ecspi4_pads));
	// enable ecspi4_clk
	setbits_le32(&ccm_regs->CCGR1, MXC_CCM_CCGR1_ECSPI4S_MASK);
	slave = spi_setup_slave(DA9063_SPI_BUS, DA9063_SPI_CS, 1000000, SPI_MODE_0);
	if (!slave)
		return -1;
	spi_claim_bus(slave);

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
	if (dev_id != 0x61 ||
	    var_id != 0x63) {
		printf("PMIC DA90631 detected wrong device");
		spi_release_bus(slave);
		return -1;
	}

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

	/* 1V2 is an acceptable level up to 800 MHz */
	if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x5A) ||
	    pmic_write_reg(DA9063_REG_VBCORE1_B, 0x5A))
		printf("Could not configure VBCORE1 voltage to 1V2\n");
	if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x5A) ||
	    pmic_write_reg(DA9063_REG_VBCORE2_B, 0x5A))
		printf("Could not configure VBCORE2 voltage to 1V2\n");
	if (pmic_write_reg(DA9063_REG_LDO5_CONT, 1))
		printf("Could not configure DA9063_REG_LDO5_CONT to on\n");
#endif
	// Disable OTP-enabled LDO4. Not connected in ec702
	if (pmic_write_bitfield(DA9063_REG_LDO4_CONT, DA9063_LDO_EN, 0))
		log_err("Failed to disable LDO4\n");

	spi_release_bus(slave);
	return 0;
}
#endif

#if defined(CONFIG_OF_BOARD_SETUP)

/*
 * Platform function to modify the FDT as needed
 * Invocation triggered by CONFIG_OF_BOARD_SETUP
 */
int ft_board_setup(void *blob, struct bd_info *bd)
{
	if (IS_ENABLED(CONFIG_VIDEO_IPUV3)) {
		int temp[2];

		temp[0] = cpu_to_fdt32(gd->fb_base);
		temp[1] = cpu_to_fdt32(640 * 480 * 2);
		printf("%s base=%i, size=%i\n", __func__, temp[0], temp[1]);
		do_fixup_by_path(blob, "/fb@0", "bootlogo", temp, sizeof(temp), 0);
	}

#if defined(CONFIG_CMD_UPDATE_FDT_EEPROM)
	patch_fdt_eeprom(blob);
#endif
	return 0;
}
#endif /* CONFIG_OF_BOARD_SETUP */

/*
 * Override for the weak definition in mxc_spi
 * Note: bus is 0-indexed, in schematic it is 1-indexed
 */
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	switch (bus) {
	case 0:
		if (cs == 0)
			return IMX_GPIO_NR(5, 28); // SPI1_CS1_n
		else if (cs == 1)
			return IMX_GPIO_NR(5, 29); // SPI1_IRDM_CS_n
		break;
	case 3:
		if (cs == 0)
			return IMX_GPIO_NR(3, 20); // DA9063 CS
		break;
	default:
		log_err("%s: Invalid bus (%u)\n", __func__, bus);
		return -1;
	}
	log_err("%s: Invalid CS (%u)\n", __func__, cs);
	return -1;
}

#ifdef CONFIG_ENV_IS_IN_MMC
/*
 * Override for weak platform function
 */
int board_mmc_get_env_dev(int devno)
{
	return CONFIG_SYS_MMC_ENV_DEV;
}

static int check_mmc_autodetect(void)
{
	char *autodetect_str = env_get("mmcautodetect");

	if (autodetect_str && (strcmp(autodetect_str, "yes") == 0))
		return 1;

	return 0;
}

/* This should be defined for each board */
int mmc_map_to_kernel_blk(int dev_no)
{
	return CONFIG_SYS_MMC_ENV_DEV;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_dev();

	if (!check_mmc_autodetect())
		return;

	env_set_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	env_set("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

#ifdef CONFIG_FSL_ESDHC_IMX
#if !CONFIG_IS_ENABLED(DM_MMC)
static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D4__SD2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D5__SD2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D6__SD2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D7__SD2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(struct bd_info *bis)
{
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned int reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x1:
		SETUP_IOMUX_PADS(usdhc2_pads);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x2:
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}
#endif
#endif

static int ar8031_phy_fixup(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	if (!is_mx6dqp()) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

		val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
		val &= 0xffe3;
		val |= 0x18;
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);
	}

	/* set the IO voltage to 1.8v */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	ar8031_phy_fixup(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)

static int detect_truly(struct display_info_t const *dev)
{
	return 1;
}

static void backlight_on(bool on)
{
	struct udevice *pwm_dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_PWM, "pwm@2080000", &pwm_dev);
	if (ret) {
		log_err("%s: pwm_init failed '%d'\n", __func__, ret);
		return;
	}

	/* Set to 70% duty cycle as in linux */
	ret = pwm_set_config(pwm_dev, 0, 500000, 350000);
	if (ret) {
		log_err("%s: pwm_set_config failed '%d'\n", __func__, ret);
		return;
	}

	pwm_set_enable(pwm_dev, 0, on);
}

static void enable_backlight(struct display_info_t const *dev)
{
	backlight_on(true);
}

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.di = 0,
	.detect	= detect_truly,
	.enable	= enable_backlight,
	.mode	= {
		.name           = "TRULY-VGA",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 33000,
		.left_margin    = 150,
		.right_margin   = 100,
		.upper_margin   = 16,
		.lower_margin   = 16,
		.hsync_len      = 90,
		.vsync_len      = 4,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED,
		.flag           = 0
		}
	}
};

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	enable_ipu_clock();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK |
		 MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) |
	       (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
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

#ifdef CONFIG_FEC_MXC
static void setup_fec(void)
{
	if (is_mx6dqp()) {
		int ret;

		/* select ENET MAC0 TX clock from PLL */
		imx_iomux_set_gpr_register(5, 9, 1, 1);
		ret = enable_fec_anatop_clock(0, ENET_125MHZ);
		if (ret)
			printf("Error fec anatop clock settings!\n");
	}
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		/*
		 * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}
	return 0;
}
#endif

// Initialize boot timer
void board_setup_timer(void)
{
	struct epit *epit_regs = (struct epit *)EPIT1_BASE_ADDR;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg;

	clrbits_le32(&epit_regs->cr, 0x00000001);   // Disable
	// Root clock is 49.5 MHz. 0x30 is a prescaler of 49.
	// 49.5 MHz / 49 = 1.01 MHz. This is the closest we can get to 1 MHz
	setbits_le32(&epit_regs->cr, 0x012C0302);   // 1 MHz free running no output
	setbits_le32(&epit_regs->cr, 0x00000001);   // Enable

	reg = readl(&mxc_ccm->CCGR1);
	reg |= MXC_CCM_CCGR1_EPIT1S_MASK;
	writel(reg, &mxc_ccm->CCGR1);
}

int board_early_init_f(void)
{
	board_setup_timer();
	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
}
#endif

static void platform_viewfinder_power_set(bool enable)
{
	struct gpio_desc disp_pwr_en_desc;
	int ret;

	ret = dm_gpio_lookup_name("gpio@23_6", &disp_pwr_en_desc);
	if (ret) {
		log_err("%s lookup gpio@23_6 failed with status %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&disp_pwr_en_desc, "DISP_PWR_EN");
	if (ret) {
		log_err("%s request DISP_PWR_EN failed with status %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&disp_pwr_en_desc, GPIOD_IS_OUT);
	dm_gpio_set_value(&disp_pwr_en_desc, enable);
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_DM_REGULATOR)
	regulators_enable_boot_on(false);
#endif

#ifdef CONFIG_MXC_SPI
	platform_setup_pmic_voltages();
#endif
	platform_viewfinder_power_set(true);

#ifdef CONFIG_FLIR_USBCHARGE
	usb_charge_setup();
#else
	log_info("FLIR_USBCHARGE is not configured\n");
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	if (IS_ENABLED(CONFIG_VIDEO_IPUV3)) {
		struct mipi_dsi_ops ops;

		setup_display();
		ops.get_lcd_videomode = mipid_st7703_get_lcd_videomode;
		ops.lcd_setup = mipid_st7703_lcd_setup;

		mxc_mipi_dsi_enable(&ops);
	}

	return 0;
}


int board_late_init(void)
{

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EC701");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

	run_command("mw.l 20e0154 5 1", 0); //set pinmux for GPIO5.0 to gpio pin

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	setup_spinor();
	return 0;
}

