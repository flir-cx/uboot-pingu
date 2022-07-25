// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 */

#include <image.h>
#include <init.h>
#include <net.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/global_data.h>
#include <asm/mach-imx/spi.h>
#include <spi.h>
#include <env.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <miiphy.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/arch/mx6-ddr.h>
#include <power/regulator.h>
#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#ifdef CONFIG_FSL_FASTBOOT
#include <fb_fsl.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

// #include <cmd_loadfpga.h>
#include "../../../flir/include/cmd_loadfpga.h"
#include "../../../flir/include/da9063.h"
#include "../../../flir/include/da9063_regs.h"
#include "../../../include/configs/platform.h"
#include "../../../board/flir/ec101/ec101.h"
#include "../../../flir/include/board_support.h"

DECLARE_GLOBAL_DATA_PTR;

char *get_last_reset_cause(void);
void imx_bypass_ldo(void);
struct spi_slave *slave;
int setup_pmic_voltages(void);
int platform_setup_pmic_voltages(void);
int fpga_power(bool enable);
int pwm_init(int pwm_id, int div, int invert);
int pwm_config(int pwm_id, int duty_ns, int period_ns);
void pwm_enable(int pwm_id);

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define KEY_VOL_UP	IMX_GPIO_NR(1, 4)

//default hw support
static struct hw_support hardware =
{
	.mipi_mux =	false,
	.display = true,
	.usb_charge = false,
	.name = "Unknown Camera"
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t const ecspi1_pads[] = {
    MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
    MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
}
#endif

static iomux_v3_cfg_t const rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 1) {
		if(cs == 0) {
			return IMX_GPIO_NR(5, 28); //SPI1_CS0_n
		} else if(cs == 1) {
			return IMX_GPIO_NR(5, 29); //SPI1_CS1_n
		}
	} else if (bus == 3) {
		if(cs == 0) {
			return IMX_GPIO_NR(3, 20); //DA9063 CS
		}
	}

	return -1;
}

static iomux_v3_cfg_t const bl_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
	struct gpio_desc desc;
	int ret;

	SETUP_IOMUX_PADS(bl_pads);

	ret = dm_gpio_lookup_name("GPIO1_21", &desc);
	if (ret)
		return;

	ret = dm_gpio_request(&desc, "Display Power Enable");
	if (ret)
		return;

	dm_gpio_set_dir_flags(&desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);
	enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
	enable_backlight();
}

#ifdef CONFIG_ENV_IS_IN_MMC
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

#ifdef CONFIG_SYS_I2C
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};
#endif

#if defined(CONFIG_PCIE_IMX) && !defined(CONFIG_DM_PCI)
iomux_v3_cfg_t const pcie_pads[] = {
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* POWER */
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* RESET */
};

static void setup_pcie(void)
{
	SETUP_IOMUX_PADS(pcie_pads);
	gpio_request(CONFIG_PCIE_IMX_POWER_GPIO, "PCIE Power Enable");
	gpio_request(CONFIG_PCIE_IMX_PERST_GPIO, "PCIE Reset");
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

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
static iomux_v3_cfg_t const epdc_enable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__EPDC_DATA00	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA10__EPDC_DATA01	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA12__EPDC_DATA02	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA11__EPDC_DATA03	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_LBA__EPDC_DATA04	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB2__EPDC_DATA05	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_CS0__EPDC_DATA06	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_RW__EPDC_DATA07	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A21__EPDC_GDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A22__EPDC_GDSP	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A23__EPDC_GDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A24__EPDC_GDRL	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__EPDC_SDCLK_P	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__EPDC_SDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA1__EPDC_SDLE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB1__EPDC_SDSHR	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA2__EPDC_BDR0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA4__EPDC_SDCE0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA5__EPDC_SDCE1	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA6__EPDC_SDCE2	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__GPIO2_IO22),
	IOMUX_PADS(PAD_EIM_DA10__GPIO3_IO10),
	IOMUX_PADS(PAD_EIM_DA12__GPIO3_IO12),
	IOMUX_PADS(PAD_EIM_DA11__GPIO3_IO11),
	IOMUX_PADS(PAD_EIM_LBA__GPIO2_IO27),
	IOMUX_PADS(PAD_EIM_EB2__GPIO2_IO30),
	IOMUX_PADS(PAD_EIM_CS0__GPIO2_IO23),
	IOMUX_PADS(PAD_EIM_RW__GPIO2_IO26),
	IOMUX_PADS(PAD_EIM_A21__GPIO2_IO17),
	IOMUX_PADS(PAD_EIM_A22__GPIO2_IO16),
	IOMUX_PADS(PAD_EIM_A23__GPIO6_IO06),
	IOMUX_PADS(PAD_EIM_A24__GPIO5_IO04),
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31),
	IOMUX_PADS(PAD_EIM_D27__GPIO3_IO27),
	IOMUX_PADS(PAD_EIM_DA1__GPIO3_IO01),
	IOMUX_PADS(PAD_EIM_EB1__GPIO2_IO29),
	IOMUX_PADS(PAD_EIM_DA2__GPIO3_IO02),
	IOMUX_PADS(PAD_EIM_DA4__GPIO3_IO04),
	IOMUX_PADS(PAD_EIM_DA5__GPIO3_IO05),
	IOMUX_PADS(PAD_EIM_DA6__GPIO3_IO06),
};
#endif

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
	unsigned reg = readl(&psrc->sbmr1) >> 11;
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

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
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

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 800,
	.vl_row = 600,
	.vl_pixclock = 26666667,
	.vl_left_margin = 8,
	.vl_right_margin = 100,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 4,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	.cmap = 0,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 419,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 5,
	.num_ce = 1,
};

static iomux_v3_cfg_t const epdc_pwr_ctrl_pads[] = {
	IOMUX_PADS(PAD_EIM_A17__GPIO2_IO21	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__GPIO3_IO17	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A18__GPIO2_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

struct gpio_desc epd_pwrstat_desc;
struct gpio_desc epd_vcom_desc;
struct gpio_desc epd_wakeup_desc;
struct gpio_desc epd_pwr_ctl0_desc;

static void setup_epdc_power(void)
{
	int ret;

	SETUP_IOMUX_PADS(epdc_pwr_ctrl_pads);

	/* Setup epdc voltage */

	/* EIM_A17 - GPIO2[21] for PWR_GOOD status */
	/* Set as input */
	ret = dm_gpio_lookup_name("GPIO2_21", &epd_pwrstat_desc);
	if (ret) {
		printf("%s lookup GPIO2_21 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_pwrstat_desc, "EPDC PWRSTAT");
	if (ret) {
		printf("%s request EPDC PWRSTAT failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_pwrstat_desc, GPIOD_IS_IN);

	/* EIM_D17 - GPIO3[17] for VCOM control */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO3_17", &epd_vcom_desc);
	if (ret) {
		printf("%s lookup GPIO3_17 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_vcom_desc, "EPDC VCOM0");
	if (ret) {
		printf("%s request EPDC VCOM0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_vcom_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* EIM_D20 - GPIO3[20] for EPD PMIC WAKEUP */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO3_20", &epd_wakeup_desc);
	if (ret) {
		printf("%s lookup GPIO3_20 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_wakeup_desc, "EPDC PWR WAKEUP");
	if (ret) {
		printf("%s request EPDC PWR WAKEUP failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_wakeup_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	/* Set as output */
	ret = dm_gpio_lookup_name("GPIO2_20", &epd_pwr_ctl0_desc);
	if (ret) {
		printf("%s lookup GPIO2_20 failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_request(&epd_pwr_ctl0_desc, "EPDC PWR CTRL0");
	if (ret) {
		printf("%s request EPDC PWR CTRL0 failed ret = %d\n", __func__, ret);
		return;
	}

	dm_gpio_set_dir_flags(&epd_pwr_ctl0_desc, GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	SETUP_IOMUX_PADS(epdc_enable_pads);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO */
	SETUP_IOMUX_PADS(epdc_disable_pads);
}

static void setup_epdc(void)
{
	unsigned int reg;
	struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk (IPU2_CLK) from PFD_400M, set to 396/2 = 198MHz */
	reg = readl(&ccm_regs->cscdr3);
	reg &= ~0x7C000;
	reg |= (1 << 16) | (1 << 14);
	writel(reg, &ccm_regs->cscdr3);

	/* EPDC AXI clk enable */
	reg = readl(&ccm_regs->CCGR3);
	reg |= 0x00C0;
	writel(reg, &ccm_regs->CCGR3);

	/* EPDC PIX clk (IPU2_DI1_CLK) from PLL5, set to 650/4/6 = ~27MHz */
	reg = readl(&ccm_regs->cscdr2);
	reg &= ~0x3FE00;
	reg |= (2 << 15) | (5 << 12);
	writel(reg, &ccm_regs->cscdr2);

	/* PLL5 enable (defaults to 650) */
	reg = readl(&ccm_regs->analog_pll_video);
	reg &= ~((1 << 16) | (1 << 12));
	reg |= (1 << 13);
	writel(reg, &ccm_regs->analog_pll_video);

	/* EPDC PIX clk enable */
	reg = readl(&ccm_regs->CCGR3);
	reg |= 0x0C00;
	writel(reg, &ccm_regs->CCGR3);

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();
}

void epdc_power_on(void)
{
	unsigned int reg;
	struct gpio_regs *gpio_regs = (struct gpio_regs *)GPIO2_BASE_ADDR;

	/* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
	dm_gpio_set_value(&epd_pwr_ctl0_desc, 1);
	udelay(1000);

	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set PMIC Wakeup to high - enable Display power */
	dm_gpio_set_value(&epd_wakeup_desc, 1);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(&gpio_regs->gpio_psr);
		if (!(reg & (1 << 21)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	dm_gpio_set_value(&epd_vcom_desc, 1);

	udelay(500);
}

void epdc_power_off(void)
{
	/* Set PMIC Wakeup to low - disable Display power */
	dm_gpio_set_value(&epd_wakeup_desc, 0);

	/* Disable VCOM */
	dm_gpio_set_value(&epd_vcom_desc, 0);

	epdc_disable_pins();

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	dm_gpio_set_value(&epd_pwr_ctl0_desc, 0);
}
#endif

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

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0,
	//.pixfmt	= IPU_PIX_FMT_BGR24,   //bpp = 24, FLIR logo
	.pixfmt	= IPU_PIX_FMT_RGB24,   //bpp = 24, FLIR logo
	//.pixfmt	= IPU_PIX_FMT_RGB565,
	//.pixfmt	= IPU_PIX_FMT_GENERIC,   //bpp = 8, Freescale-NXP logo
	.di = 0,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "KOPIN-VGA",
		.refresh        = 60,
		.xres           = 640,  
		.yres           = 480,
		//.pixclock       = 50000, // When using Lauterbach T32 to load the image
		.pixclock       = 37000,   // When loading the imasge over USB using imx_usb 
		.left_margin    = 100,
		.right_margin   = 100,
		.upper_margin   = 31,
		.lower_margin   = 10,
                .hsync_len      = 96,
		.vsync_len      = 4,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED,
        	.flag           = 0
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15384,
		.left_margin    = 160,
		.right_margin   = 24,
		.upper_margin   = 29,
		.lower_margin   = 3,
		.hsync_len      = 136,
		.vsync_len      = 6,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "SEIKO-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 23,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
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
	setbits_le32(&epit_regs->cr, 0x012C0412);   // 1 MHz free running no output
	setbits_le32(&epit_regs->cr, 0x00000001);   // Enable

	reg = readl(&mxc_ccm->CCGR1);
	reg |= MXC_CCM_CCGR1_EPIT1S_MASK;
	writel(reg, &mxc_ccm->CCGR1);
}

static void dm_start_i2c(int early)
{
	const struct dm_i2c_ops *i2c_ops = NULL;
	struct udevice *i2c_devp = NULL;

	if (uclass_get_device_by_name(UCLASS_I2C, "i2c@21f8000", &i2c_devp) == -ENODEV)
	{
	    if (!early)
		printf("%s, %s, %d: dev i2c@21f8000 not found!\n", __FILE__, __FUNCTION__, __LINE__);
		return;
	}
	i2c_ops = device_get_ops(i2c_devp);
	if (i2c_ops == NULL)
	    return;

	if (hardware.mipi_mux)
	{
	    unsigned char buf[2];
	    struct i2c_msg msg;

	    msg.addr  = LEIF_PCA9534_ADDRESS;
	    msg.flags = 0; // Write
	    msg.len   = 1;

	    //set LCD_MIPI_SEL=1 and LCD_MIPI_EN=0
	    buf[0] = 0xbf;
	    msg.buf = buf;
	    i2c_ops->xfer(i2c_devp, &msg, 1);

	    buf[0] = 0x9f;
	    msg.buf = buf;
	    i2c_ops->xfer(i2c_devp, &msg, 1);
	}
}

static void setup_pwm_n_mipi_dsi(int early)
{
	//init backlight to lcd
	pwm_init(PWM1, 0, 0);
	//duty cycle = 70%, period = 0.2ms (should match duty cycle in kernel)
	pwm_config(PWM1, 140000, 200000);
	pwm_enable(PWM1); 
	dm_start_i2c(early);
	mxc_mipi_dsi_enable(early);
}

int board_early_init_f(void)
{
        board_setup_timer();
        setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
	setup_pwm_n_mipi_dsi(1);
#endif

	return 0;
}

int board_early_init_f_rest(void)
{
#if defined(CONFIG_VIDEO_IPUV3)
	struct udevice *ipu_devp = NULL;
	uclass_get_device_by_name(UCLASS_VIDEO, "ipu@2400000", &ipu_devp);
#endif
	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{

}
#endif

int checkboard(void)
{
    printf("Board: %s - %s\n", CONFIG_BOARD_DESCRIPTION, hardware.name);
	return 0;
}

int board_init(void)
{
        int ret = 0;
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_DM_REGULATOR)
       	regulators_enable_boot_on(false);
#endif

#ifdef CONFIG_MXC_SPI
	ret = platform_setup_pmic_voltages();
	// ret = setup_pmic_voltages();
       	if (ret)
		return -1;
	setup_spi();
#endif

#ifdef CONFIG_SYS_I2C
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	/* Setup I2C4  */
	ret = setup_i2c(3, CONFIG_SYS_I2C_SPEED,
		CONFIG_SYS_I2C_SLAVE, &i2c_pad_info1);
	/* Setup I2C3 */
	ret = setup_i2c(2, CONFIG_SYS_I2C_SPEED,
		CONFIG_SYS_I2C_SLAVE, &i2c_pad_info2);
#endif

#ifdef CONFIG_SYS_I2C_MXC
	ret = setup_pmic_voltages();
	if (ret)
		return ret;

	struct Eeprom ioboard =
	{
	 .i2c_bus = 2,
	 .i2c_address = 0xaa,
	 .i2c_offset = 0x0,
	 //.article = 0,
	 //.revision = 0,
	 //.name = "\0",
	};


	board_support_setup(&ioboard, &hardware);
#endif
	
#if defined(CONFIG_PCIE_IMX) && !defined(CONFIG_DM_PCI)
	setup_pcie();
#endif

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	setup_epdc();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

#if defined(CONFIG_VIDEO_IPUV3)
	if (hardware.display) 
	{
	    setup_pwm_n_mipi_dsi(0);
	}
#endif

	return 0;
}

#ifdef CONFIG_POWER
int power_init_board(void)
{
	struct pmic *pfuze;
	unsigned int reg;
	int ret;

	pfuze = pfuze_common_init(I2C_PMIC);
	if (!pfuze)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(pfuze, APS_APS);
	else
		ret = pfuze_mode_init(pfuze, APS_PFM);

	if (ret < 0)
		return ret;
	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		pmic_reg_read(pfuze, PFUZE100_VGEN3VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(pfuze, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		pmic_reg_read(pfuze, PFUZE100_VGEN5VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(pfuze, PFUZE100_VGEN5VOL, reg);
	}

	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW2STBY, &reg);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(pfuze, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW2CONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}

#elif defined(CONFIG_DM_PMIC_PFUZE100)
int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg;
	int ret;

	dev = pfuze_common_init();
	if (!dev)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(dev, APS_APS);
	else
		ret = pfuze_mode_init(dev, APS_PFM);
	if (ret < 0)
		return ret;

	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN3VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(dev, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);
	}

	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW2STBY);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(dev, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW2CONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1ABSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1ABCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}
#endif

#ifdef CONFIG_LDO_BYPASS_CHECK
#ifdef CONFIG_POWER
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	unsigned char vddarm;
	struct pmic *p = pmic_get("PFUZE100");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x29;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			value |= 0x2d;
			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.425V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x2d;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x1c;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			if (is_mx6dl())
				value |= 0x27;
			else
				value |= 0x20;

			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.3V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x28;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.15V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			if (is_400M)
				value |= 0x17;
			else
				value |= 0x1e;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		}

		if (is_400M) {
			if (is_mx6dl())
				vddarm = 0x22;
			else
				vddarm = 0x1b;
		} else {
			if (is_mx6dl())
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE100_SW1ABVOL, value);

		/* decrease VDDSOC to 1.175V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x23;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#elif defined(CONFIG_DM_PMIC_PFUZE100)
void ldo_mode_set(int ldo_bypass)
{
	int is_400M;
	unsigned char vddarm;
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze100@8", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0; /* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x29);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x2d);
		}
		/* increase VDDSOC to 1.425V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x2d);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1c);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			if (is_mx6dl())
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x27);
			else
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x20);
		}
		/* increase VDDSOC to 1.3V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x28);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.15V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			if (is_400M)
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x17);
			else
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1e);
		}

		if (is_400M) {
			if (is_mx6dl())
				vddarm = 0x22;
			else
				vddarm = 0x1b;
		} else {
			if (is_mx6dl())
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, vddarm);

		/* decrease VDDSOC to 1.175V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x23);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	
#ifdef CONFIG_SYS_USE_SPINOR
       	setup_spinor();
#endif
	env_set("tee", "no");
#ifdef CONFIG_IMX_OPTEE
	env_set("tee", "yes");
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EVCO");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}


#if defined(CONFIG_OF_BOARD_SETUP)

/* Platform function to modify the FDT as needed */
int ft_board_setup(void *blob, struct bd_info *bd)
{
	uchar enetaddr[6];
	//fix ethernet mac-address using direct path to node
	eth_env_get_enetaddr("ethaddr", enetaddr);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000", "mac-address", &enetaddr, 6, 1);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000", "local-mac-address", &enetaddr, 6, 1);

	/*	do_fixup_by_path_string(blob, "/u-boot", "version", U_BOOT_VERSION_STRING);
	do_fixup_by_path_string(blob, "/u-boot", "reset-cause", get_last_reset_cause());
	*/

#if defined(CONFIG_VIDEO_IPUV3)
    int temp[2];
    temp[0] = cpu_to_fdt32(gd->fb_base);
    temp[1] = cpu_to_fdt32(640*480*2);
    do_fixup_by_path(blob, "/fb@0", "bootlogo", temp, sizeof(temp), 0);
#endif

#if defined(CONFIG_CMD_UPDATE_FDT_EEPROM)
    patch_fdt_eeprom(blob);
#endif
	return 0;
}

int platform_setup_pmic_voltages(void)
{
    unsigned char dev_id, var_id, cust_id, conf_id;
    struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

    gpio_request(IMX_GPIO_NR(3, 20), "CS SPI4");
    gpio_direction_output(IMX_GPIO_NR(3, 20),1);
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
    printf("PMIC (%s):  DA9063, Device: 0x%02x, Variant: 0x%02x, "
           "Customer: 0x%02x, Config: 0x%02x\n", __FUNCTION__, dev_id, var_id,
           cust_id, conf_id);
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
    //    imx_bypass_ldo();
    /* 1V2 is an acceptable level up to 800 MHz */
    if (pmic_write_reg(DA9063_REG_VBCORE1_A, 0x5A) ||
	pmic_write_reg(DA9063_REG_VBCORE1_B, 0x5A))
	    printf("Could not configure VBCORE1 voltage to 1V2\n");
    if (pmic_write_reg(DA9063_REG_VBCORE2_A, 0x5A) ||
	pmic_write_reg(DA9063_REG_VBCORE2_B, 0x5A))
	    printf("Could not configure VBCORE2 voltage to 1V2\n");
#endif
    /* gpio_free(IMX_GPIO_NR(3,20)); */

    spi_release_bus(slave);
    setup_spi();
    return 0;
}

int setup_pmic_voltages(void)
{
    unsigned char dev_id, var_id, cust_id, conf_id;
    struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

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
    printf("PMIC (%s):  DA9063, Device: 0x%02x, Variant: 0x%02x, "
           "Customer: 0x%02x, Config: 0x%02x\n", __FUNCTION__, dev_id, var_id,
           cust_id, conf_id);

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
	spi_release_bus(slave);
	return 0;
	
}

#endif /* CONFIG_OF_BOARD_SETUP */
