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
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <fdt_support.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <pwm.h>
#include "../../freescale/common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/arch/mx6-ddr.h>
#include <power/regulator.h>
#include <spi.h>
#include <splash.h>
#include "../../../drivers/video/mxc_mipi_dsi.h"
#include "../../../drivers/video/mxcfb_otm1287.h"
#include "../../../drivers/video/mxcfb_st7703.h"
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

#include "../common/cmd_loadfpga.h"
#include "../common/da9063.h"
#include "../common/da9063_regs.h"
#include "../common/board_support.h"
#include "../common/usbcharge.h"
#include "../common/fpga_ctrl.h"

DECLARE_GLOBAL_DATA_PTR;
#define LOG_DEBUG
struct spi_slave *slave;

#if defined(CONFIG_IMX6_LDO_BYPASS)
void imx_bypass_ldo(void);
#endif
static int setup_pmic_voltages(void);
static int platform_setup_pmic_voltages(void);
int fpga_power(bool enable);

static void setup_mipi_mux_i2c(void);

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

#define LEIF_PCA9534_ADDRESS (0x4a >> 1)
#define LEIF_TP_EN_PIN (4)
#define EVIO_PCA9534_ADDRESS (0x46 >> 1)
#define EVIO_TP_EN_PIN (7)

#define PWM1 0

#define GPIO_SPI1_SCLK     IMX_GPIO_NR(5, 22)
#define GPIO_SPI1_MOSI     IMX_GPIO_NR(5, 23)
#define GPIO_SPI1_MISO     IMX_GPIO_NR(5, 24)
#define GPIO_SPI1_CS       IMX_GPIO_NR(5, 28)

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi4_pads[] = {
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D20__GPIO3_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const no_ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__GPIO5_IO22  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__GPIO5_IO23  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__GPIO5_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

//default hw support
static struct hw_support hardware = {
	.mipi_mux = false,
	.display = true,
	.usb_charge = IS_ENABLED(CONFIG_FLIR_USBCHARGE),
	.name = "Unknown Camera"
};



/**
 * @brief Overrides the (weak) splash_screen_prepare in splash.c
 *
 * @return int non zero on error
 */
int splash_screen_prepare(void)
{
	char *env_loadsplash;

	// IF_DEFINED(CONFIG_FLIR_USBCHARGE) >>>> NOT POPSSIBLE <<<<,
	// this is a compile time issue and not a run time one!!
#ifdef CONFIG_FLIR_USBCHARGE
	set_boot_logo();
#else
	log_info("Cannot set boot logo!\n");
	log_info("This u-boot was not built with CONFIG_FLIR_USBCHARGE=y!\n");
#endif

	if (!env_get("splashimage")) {
		log_err("Environment variable splashimage not found!\n");
		return -EINVAL;
	}

	env_loadsplash = env_get("loadsplash");
	if (env_loadsplash == NULL) {
		log_err("Environment variable loadsplash not found!\n");
		return -EINVAL;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		log_err("Failed to run loadsplash %s\n\n", env_loadsplash);
		return -ENOSYS;
	}
	return 0;
}

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

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0) {
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

/**
 * @brief Get the touchpad expander i2c addr
 *
 * @return u8
 */
static u8 get_tp_expander_addr(void)
{
	if (hardware.mipi_mux) // Lennox Camera
		return LEIF_PCA9534_ADDRESS;
	return EVIO_PCA9534_ADDRESS;
}

/**
 * @brief Get the touchpad enable mask on the io expander
 *
 * @return u8
 */
static u8 get_tp_expander_mask(void)
{
	if (hardware.mipi_mux) // Lennox Camera
		return (1 << LEIF_TP_EN_PIN);
	return (1 << EVIO_TP_EN_PIN);
}

/**
 * @brief Detects Orise panel by probing the touch located on bus 3,
 *	  address 0x38. To probe the touch it first needs to be enabled.
 *
 * @param dev N/A
 * @return int 0 if found
 */
static int do_detect_orise(u8 expander_addr, u8 expander_mask)
{
	struct udevice *bus2, *bus3, *pwrdev, *touchdev;
	int ret;
	unsigned int val;

	/* First enable the touch, O7 of PCA9534BS on bus 2 address 0x23, reg 3 */
	ret = uclass_get_device_by_name(UCLASS_I2C, "i2c@21a8000", &bus2);
	if (ret) {
		log_err("%s: probe pwr expander, failed on bus 2\n", __func__);
		return ret;
	}

	ret = dm_i2c_probe(bus2, expander_addr, DM_I2C_CHIP_RD_ADDRESS |
				  DM_I2C_CHIP_WR_ADDRESS, &pwrdev);
	if (ret) {
		log_err("%s: probe pwr expander, failed on device %d\n",
			__func__, expander_addr);
		return ret;
	}

	ret = dm_i2c_reg_read(pwrdev, 3);
	if (ret < 0)
		return ret;
	val = ret;
	val &= ~expander_mask;
	ret = dm_i2c_reg_write(pwrdev, 3, val);

	/* Wait 200 ms, from data sheet, for touch controller to start up. */
	mdelay(200);

	/* Try to probe the actual touch device, bus 3 addr 0x38 */
	ret = uclass_get_device_by_name(UCLASS_I2C, "i2c@21f8000", &bus3);
	if (ret) {
		log_info("%s: Orise display probed, not found on bus 3\n", __func__);
		goto touch_enable_restore;
	}

	ret = dm_i2c_probe(bus3, 0x38, DM_I2C_CHIP_RD_ADDRESS |
				  DM_I2C_CHIP_WR_ADDRESS, &touchdev);

touch_enable_restore:
	/* Turn off the touch again by setting enable to input on pwr expander */
	val |= 0x80;
	dm_i2c_reg_write(pwrdev, 3, val);

	return ret;
}

/**
 * @brief Detects Orise panel by probing the touch located
 *	  on bus 3, address 0x38
 *
 * @param dev N/A
 * @return int 0 if not found
 */
static int detect_orise(struct display_info_t const *dev)
{
	static int cache_ret = -ENOTCONN;

	if (cache_ret == -ENOTCONN) {
		u8 expander_addr = get_tp_expander_addr();
		u8 expander_mask = get_tp_expander_mask();

		cache_ret = do_detect_orise(expander_addr, expander_mask);
	}

	return (cache_ret == 0);
}

/**
 * @brief Detects Truly panel by probing the touch located
 *	  on bus 3, address 0x70
 *
 * @param dev N/A
 * @return int 0 if found
 */
static int do_detect_truly(void)
{
	struct udevice *bus, *touchdevice;
	int ret;

	/* Try to probe the actual touch device, bus 3 addr 0x70 */
	ret = uclass_get_device_by_name(UCLASS_I2C, "i2c@21f8000", &bus);
	if (ret) {
		log_info("%s: Truly display probed, not found on bus %d\n",
			 __func__, 3);
		return ret;
	}

	return dm_i2c_probe(bus, 0x70, DM_I2C_CHIP_RD_ADDRESS |
				DM_I2C_CHIP_WR_ADDRESS, &touchdevice);
}

/**
 * @brief Detects Truly panel by probing the touch located
 *	  on bus 3, address 0x70
 *
 * @param dev N/A
 * @return int 0 if not found
 */
static int detect_truly(struct display_info_t const *dev)
{
	static int cache_ret = -ENOTCONN;

	if (cache_ret == -ENOTCONN)
		cache_ret = do_detect_truly();

	return (cache_ret == 0);
}

void backlight_on(bool on)
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

struct display_info_t const displays[] = {{
	.bus	= 1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.di = 0,
	.detect	= detect_orise,
	.enable	= enable_backlight,
	.mode	= {
		.name           = "ORISE-VGA",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 37000,
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
} }
};
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();

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

static void setup_mipi_mux_i2c()
{
	const struct dm_i2c_ops *i2c_ops = NULL;
	struct udevice *i2c_devp = NULL;

	if (hardware.mipi_mux)
	{
		unsigned char buf[2];
		struct i2c_msg msg;
		if (uclass_get_device_by_name(UCLASS_I2C, "i2c@21f8000", &i2c_devp) == -ENODEV) {
			printf("%s, %s, %d: dev i2c@21f8000 not found!\n", __FILE__, __FUNCTION__, __LINE__);
			return;
		}
		i2c_ops = device_get_ops(i2c_devp);
		if (i2c_ops == NULL) {
			log_err("Failed to get i2c_ops from device at i2c@21f8000\n");
			return;
		}

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

int board_early_init_f(void)
{
	board_setup_timer();
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
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

	if (IS_ENABLED(CONFIG_MXC_SPI)) {
		ret = platform_setup_pmic_voltages();
		// ret = setup_pmic_voltages();
		if (ret)
			return -1;
		setup_spi();
	}

#ifdef CONFIG_SYS_I2C
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	/* Setup I2C4  */
	ret = setup_i2c(3, CONFIG_SYS_I2C_SPEED,
		CONFIG_SYS_I2C_SLAVE, &i2c_pad_info1);
	/* Setup I2C3 */
	ret = setup_i2c(2, CONFIG_SYS_I2C_SPEED,
		CONFIG_SYS_I2C_SLAVE, &i2c_pad_info2);
#endif

#ifndef CONFIG_SYS_I2C_MXC
	assert("CONFIG_SYS_I2C_MXC needs to be defined for EC101")
#endif
	ret = setup_pmic_voltages();
	if (ret)
		return ret;

	struct eeprom ioboard =
	{
	 .i2c_bus = 2,
	 .i2c_address = 0xaa,
	 .i2c_offset = 0x0,
	 //.article = 0,
	 //.revision = 0,
	 //.name = "\0",
	};


	ret = board_support_setup(&ioboard, &hardware);
	if (ret < 0) {
		printf("IO Board either missing or is not functioning!!\n");
	} else {
		// IF_DEFINED(CONFIG_FLIR_USBCHARGE) >>>> NOT POPSSIBLE <<<<,
		// this is a compile time issue and not a run time one!!
#ifdef CONFIG_FLIR_USBCHARGE
		usb_charge_setup();
#else
		log_info("This u-boot was not built with CONFIG_FLIR_USBCHARGE=y!\n");
#endif
	}

#if defined(CONFIG_PCIE_IMX) && !defined(CONFIG_DM_PCI)
	setup_pcie();
#endif

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	setup_epdc();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	if (IS_ENABLED(CONFIG_VIDEO_IPUV3)) {
		struct mipi_dsi_ops ops;
		int panel_found = 1;

		if (detect_truly(NULL)) {
			ops.get_lcd_videomode = mipid_st7703_get_lcd_videomode;
			ops.lcd_setup = mipid_st7703_lcd_setup;
			log_info("Found Truly display\n");
		} else if (detect_orise(NULL)) {
			ops.get_lcd_videomode = mipid_otm1287a_get_lcd_videomode;
			ops.lcd_setup = mipid_otm1287a_lcd_setup;
			log_info("Found ORISE display\n");
		} else {
			log_err("No panel detected, video will probably fail!\n");
			panel_found = 0;
		}

		if (panel_found) {
			setup_mipi_mux_i2c();
			mxc_mipi_dsi_enable(&ops);
		}
	}

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

	setup_spinor();

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

	if (detect_truly(NULL)) {
		/* Point to truly display instead of orise */
		do_fixup_by_path(blob, "/fb@0", "mode_str", "TRULY-VGA", 10, 0);
		do_fixup_by_path(blob, "/soc/bus@2100000/mipi@21e0000", "lcd_panel",
				 "TRULY-VGA", 10, 0);

		/* Set the status of the touch of the truly display to ok, */
		/* it should be disabled by default from the device tree. */
		do_fixup_by_path(blob, "/soc/bus@2100000/i2c@21f8000/tsc@70", "status",
				 "okay", 5, 0);
		/* Set the status of the orise touch to disabled, it should */
		/* be enabled by default from th edevice tree */
		do_fixup_by_path(blob, "/soc/bus@2100000/i2c@21f8000/edt_ft5336@38", "status",
				 "disabled", 9, 0);
	}
	if (IS_ENABLED(CONFIG_VIDEO_IPUV3)) {
		int temp[2];

		temp[0] = cpu_to_fdt32(gd->fb_base);
		temp[1] = cpu_to_fdt32(640 * 480 * 2);
		do_fixup_by_path(blob, "/fb@0", "bootlogo", temp, sizeof(temp), 0);
	}

#if defined(CONFIG_CMD_UPDATE_FDT_EEPROM)
	patch_fdt_eeprom(blob);
#endif
	return 0;
}
#endif /* CONFIG_OF_BOARD_SETUP */

#ifdef CONFIG_MXC_SPI
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
#endif /* CONFIG_MXC_SPI */

#ifdef CONFIG_SYS_I2C_MXC
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
#endif /* CONFIG_SYS_I2C_MXC */

static void ec101_fpga_set_ctrl(struct fpga_ctrl *fpga)
{
#if ! IS_ENABLED(CONFIG_FPGA_XILINX)
#error "ec101 needs to have CONFIG_FPGA_XILINX set"
#endif

	fpga->pins.program_n = IMX_GPIO_NR(5, 25);
	fpga->pins.init_n = IMX_GPIO_NR(5, 26);
	fpga->pins.done = IMX_GPIO_NR(5, 27);

	fpga_set_ops(fpga);
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
	pmic_write_bitfield(DA9063_REG_BPRO_CONT, DA9063_BUCK_EN,
			    enable ? DA9063_BUCK_EN : 0);
	// CORE_SW_EN  (1V8_FPGA)
	pmic_write_bitfield(DA9063_REG_BCORE1_CONT, DA9063_CORE_SW_EN,
			    enable ? DA9063_CORE_SW_EN : 0);
	// PERI_SW_EN    (1V2_FPGA)
	pmic_write_bitfield(DA9063_REG_BPERI_CONT, DA9063_PERI_SW_EN,
			    enable ? DA9063_PERI_SW_EN : 0);
	if(conf_id == 0x3b) //revC
	{
		// BMEM_EN         (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_BMEM_CONT, DA9063_BUCK_EN,
				    enable ? DA9063_BUCK_EN : 0);
		// LDO10_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
	}
	else //revD
	{
		// LDO10_EN          (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
		// LDO8_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO8_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
	}
	spi_release_bus(slave);
	return 0;
}

static int ec101_fpga_enable_power(struct fpga_ctrl *fpga)
{
	return fpga_power(true);
}

static int ec101_fpga_request_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//Use as cpu spi bus
	ret = gpio_request(GPIO_SPI1_CS, "spi-1-cs");
	if (ret)
		log_err("%s: gpio request failure %d\n",  __func__, ret);

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	ret = gpio_direction_output(GPIO_SPI1_CS, 1);
	if (ret)
		log_err("%s: gpio dir failure %d\n",  __func__, ret);

	return 0;
}

static int ec101_fpga_release_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//cpu spi bus conflicts with fpga spi bus, disable cpu bus
	imx_iomux_v3_setup_multiple_pads(no_ecspi1_pads,
					 ARRAY_SIZE(no_ecspi1_pads));

	ret += gpio_request(GPIO_SPI1_SCLK, "spi-1-clk");
	ret += gpio_request(GPIO_SPI1_MOSI, "spi-1-mosi");
	ret += gpio_request(GPIO_SPI1_MISO, "spi-1-miso");
	ret += gpio_direction_input(GPIO_SPI1_SCLK);
	ret += gpio_direction_input(GPIO_SPI1_MOSI);
	ret += gpio_direction_input(GPIO_SPI1_MISO);
	ret += gpio_direction_input(GPIO_SPI1_CS);

	ret += gpio_free(GPIO_SPI1_SCLK);
	ret += gpio_free(GPIO_SPI1_MOSI);
	ret += gpio_free(GPIO_SPI1_MISO);
	ret += gpio_free(GPIO_SPI1_CS);

	if (ret)
		log_err("%s: gpio failure\n",  __func__);

	return 0;
}

static void fpga_set_board_ops(struct fpga_board_ops *ops)
{
	ops->fpga_request_flash_spi = ec101_fpga_request_flash_spi;
	ops->fpga_release_flash_spi = ec101_fpga_release_flash_spi;
	ops->fpga_enable_power = ec101_fpga_enable_power;
}

void fpga_init_ctrl(struct fpga_ctrl *fpga)
{
	ec101_fpga_set_ctrl(fpga);

	fpga_set_board_ops(&fpga->board_ops);
}

static void ec101_disable_gpio(const char *name, unsigned long flags)
{
	struct gpio_desc desc;
	int ret = dm_gpio_lookup_name(name, &desc);

	if (ret < 0)
		return;
	ret = dm_gpio_request(&desc, "ioexp");
	if (ret < 0) {
		log_err("Request ioexp GPIO '%s' failed\n", name);
		return;
	}
	desc.flags = flags;
	debug("Clear %s (active %s)\n", name, flags & GPIOD_ACTIVE_LOW ? "LOW" : "HIGH");
	ret = dm_gpio_set_dir_flags(&desc, flags | GPIOD_IS_OUT);
	if (ret < 0) {
		log_err("Setting ioexp dir flags on '%s' failed\n", name);
		return;
	}
	dm_gpio_set_value(&desc, 0);
	dm_gpio_free(desc.dev, &desc);
}

/* prepare_power_off() - shut down external stuff before power-off
 *
 * Override weak function in usbcharge module
 */
void prepare_power_off(void)
{
	backlight_on(false);

	if (!strncmp(hardware.name, "Evander", 7)) {
		log_info("Evander: Additional power-off sequence\n");

		ec101_disable_gpio("usbmux@21_0", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("usbmux@21_1", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("usbmux@21_2", 0);
		ec101_disable_gpio("usbmux@21_3", 0);
		ec101_disable_gpio("usbmux@21_4", 0);
		ec101_disable_gpio("usbmux@21_5", GPIOD_ACTIVE_LOW);

		ec101_disable_gpio("pwr@23_0", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("pwr@23_1", 0);
		ec101_disable_gpio("pwr@23_2", 0);
		ec101_disable_gpio("pwr@23_3", 0);
		ec101_disable_gpio("pwr@23_4", GPIOD_ACTIVE_LOW);
		// skip CHG_OTG
		ec101_disable_gpio("pwr@23_6", 0);
		ec101_disable_gpio("pwr@23_7", GPIOD_ACTIVE_LOW);

	} else if (!strncmp(hardware.name, "Lennox", 6)) {
		log_info("Lennox: Additional power-off sequence\n");

		// USB MUX, LEDs
		ec101_disable_gpio("usbmux@21_0", GPIOD_ACTIVE_LOW);
		// skip CHG_OTG
		ec101_disable_gpio("usbmux@21_2", 0);
		ec101_disable_gpio("usbmux@21_3", 0);
		ec101_disable_gpio("usbmux@21_4", 0);
		ec101_disable_gpio("usbmux@21_5", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("usbmux@21_6", 0);
		ec101_disable_gpio("usbmux@21_7", 0);
		// skip SD1_WP

		// Audio, Autofocus, etc
		ec101_disable_gpio("pwr@23_0", 0);
		ec101_disable_gpio("pwr@23_1", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("pwr@23_2", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("pwr@23_3", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("pwr@23_4", GPIOD_ACTIVE_LOW);
		ec101_disable_gpio("pwr@23_5", 0);
		// skip GPS_SPARE
		ec101_disable_gpio("pwr@23_7", 0);

		// Viewfinder
		// skip DP_OUT
		ec101_disable_gpio("vf@25_1", 0);
		ec101_disable_gpio("vf@25_2", 0);
		// skip IO3 (NC)
		ec101_disable_gpio("vf@25_4", 0);
		ec101_disable_gpio("vf@25_5", 0);
		ec101_disable_gpio("vf@25_6", 0);
		ec101_disable_gpio("vf@25_7", 0);
	} else {
		return;
	}

	ec101_disable_gpio("erla@20_0", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_1", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_2", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_3", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_4", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_5", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_6", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("erla@20_7", GPIOD_ACTIVE_LOW);

	// EVRO board
	ec101_disable_gpio("evro@23_0", 0);
	ec101_disable_gpio("evro@23_1", 0);
	ec101_disable_gpio("evro@23_2", 0);
	ec101_disable_gpio("evro@23_3", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("evro@23_4", 0);
	ec101_disable_gpio("evro@23_5", GPIOD_ACTIVE_LOW);
	ec101_disable_gpio("evro@23_6", 0);
	ec101_disable_gpio("evro@23_7", 0);

	ec101_disable_gpio("evro@24_0", GPIOD_ACTIVE_LOW);
	// skip Spare
	ec101_disable_gpio("evro@24_2", 0);
	ec101_disable_gpio("evro@24_3", 0);
	// skip Optics_En
	ec101_disable_gpio("evro@24_5", GPIOD_ACTIVE_LOW);
}

