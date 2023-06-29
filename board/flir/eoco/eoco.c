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
#include <fdt_support.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <pwm.h>
#include "../../freescale/common/pfuze.h"
#include "../../../drivers/video/mxc_mipi_dsi.h"
#include "../../../drivers/video/mxcfb_st7703.h"
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
#include "../common/da9063.h"
#include "../common/da9063_regs.h"
#include "../common/fpga_ctrl.h"
#include "../common/cmd_loadfpga.h"
#include "../common/usbcharge.h"


DECLARE_GLOBAL_DATA_PTR;

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

#define CMD_WRITE_ENABLE 0x06
#define CMD_EN4BYTE_ADDR 0xB7
#define CMD_WRITE_ENHANCED_VOLATILE_CONF 0x61
#define SPI_FLASH_MAX_SIZE_BUF 32

struct spi_slave *slave;

/**
 * @brief Overrides the (weak) splash_screen_prepare in splash.c
 *
 * @return int non zero on error
 */
int splash_screen_prepare(void)
{
	char *env_loadsplash;

	set_boot_logo();

	if (!env_get("splashimage")) {
		log_err("Environment variable splashimage not found!\n");
		return -EINVAL;
	}

	env_loadsplash = env_get("loadsplash");
	if (!env_loadsplash) {
		log_err("Environment variable loadsplash not found!\n");
		return -EINVAL;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		log_err("Failed to run loadsplash %s\n\n", env_loadsplash);
		return -ENOENT;
	}
	return 0;
}

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
static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const no_ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__GPIO5_IO22  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__GPIO5_IO23  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__GPIO5_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const ecspi4_pads[] = {
	MX6_PAD_EIM_D28__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D22__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D21__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D20__GPIO3_IO20  | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int platform_check_pmic_boot_reason(void)
{
	spi_claim_bus(slave);

	unsigned char fault_log;

	// Check PMIC FAULT register for boot reason.
	pmic_read_reg(DA9063_REG_FAULT_LOG,&fault_log);
	pmic_write_reg(DA9063_REG_FAULT_LOG,fault_log);

	//Don't boot on Long press on-key button or watchdog timeout
	if(fault_log & (DA9063_KEY_RESET | DA9063_TWD_ERROR))
	{
		printf("Powering off....\n");

		// Power off using GPIO.
		gpio_request(IMX_GPIO_NR(2, 30), "PWR_OFF");
		gpio_direction_output(IMX_GPIO_NR(2, 30),1);

		while(1) {}
	}

	spi_release_bus(slave);
	return 0;
}

void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
	gpio_request(IMX_GPIO_NR(5, 29), "CS SPI1 1");
	gpio_direction_output(IMX_GPIO_NR(5, 29), 1);
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
    printf("PMIC:  DA9063, Device: 0x%02x, Variant: 0x%02x, "
           "Customer: 0x%02x, Config: 0x%02x\n", dev_id, var_id,
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
	spi_release_bus(slave);
	setup_spi();
	return 0;
}
#endif

#if defined(CONFIG_OF_BOARD_SETUP)

/* Platform function to modify the FDT as needed */
int ft_board_setup(void *blob, struct bd_info *bd)
{
	/*
	uchar enetaddr[6];
	//fix ethernet mac-address using direct path to node
	eth_env_get_enetaddr("ethaddr", enetaddr);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000",
			"mac-address", &enetaddr, 6, 1);
	do_fixup_by_path(blob, "/soc/aips-bus@02100000/ethernet@02188000",
			"local-mac-address", &enetaddr, 6, 1);

	do_fixup_by_path_string(blob, "/u-boot", "version", U_BOOT_VERSION_STRING);
	do_fixup_by_path_string(blob, "/u-boot", "reset-cause", get_last_reset_cause());
	*/

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

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0) {
		if(cs == 0) {
			return IMX_GPIO_NR(5, 28); //SPI1_CS0_n
		} else if(cs == 0) {
			return IMX_GPIO_NR(5, 29); //SPI1_CS1_n
		}
	}
	if (bus == 1) {
		if(cs == 0) {
			return IMX_GPIO_NR(5, 28); //SPI1_CS0_n
		} else if(cs == 0) {
			return IMX_GPIO_NR(5, 29); //SPI1_CS1_n
		}
	} else if (bus == 3) {
		if(cs == 0) {
			return IMX_GPIO_NR(3, 20); //DA9063 CS
		}
	}

	return -1;
}



static int detect_truly(struct display_info_t const *dev)
{
	return 1;
}


static void enable_truly_backlight(struct display_info_t const *dev)
{
	struct udevice *pwm_dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_PWM, "pwm@2080000", &pwm_dev);
	if (ret) {
		log_err("%s: pwm_init failed '%d'\n", __func__, ret);
		return;
	}

	// Set to 50% duty cycle
	ret = pwm_set_config(pwm_dev, 0, 500000, 250000);
	if (ret) {
		log_err("%s: pwm_set_config failed '%d'\n", __func__, ret);
		return;
	}

	pwm_set_enable(pwm_dev, 0, 1);
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

#ifdef CONFIG_DM_I2C

int platform_check_fuelgauge(void)
{
	int ret;
	unsigned char battery_level;
	struct udevice *bus, *dev;

	ret = uclass_get_device_by_seq(UCLASS_I2C, BQ40Z50_I2C_BUS, &bus);
	if (ret != 0) {
		printf("uclass_get_device_by_seq() error!\n");
		return ret;
	}

	ret = dm_i2c_probe(bus, BQ40Z50_I2C_ADDR, 0, &dev);
	if (ret == 0) {
		ret = dm_i2c_read(dev, BQ40Z50_REG_STATE_OF_CHARGE, &battery_level, 1);
		if (ret == 0) {
			printf("Battery: charge level %d%%\n", battery_level);
			if (battery_level <= BQ40Z50_BATT_CRITICAL_LEVEL) {
				printf("Battery level critical. Shuting down.\n");
				// Power off using GPIO.
				gpio_request(IMX_GPIO_NR(2, 30), "PWR_OFF");
				gpio_direction_output(IMX_GPIO_NR(2, 30), 1);
			}
		} else {
			printf("Battery read level failed.\n");
		}
	} else {
		printf("Battery missing, running on DC.\n");
	}

	return ret;
}

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
	.detect	= detect_truly,
	.enable	= enable_truly_backlight,
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
	int reg;

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

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
}
#endif

#if defined(CONFIG_VIDEO_BACKLIGHT_OFF_HANDOVER)
void set_backlight_off()
{
	struct udevice *pwm_dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_PWM, "pwm@2080000", &pwm_dev);
	if (ret) {
		log_err("%s: pwm_init failed '%d'\n", __func__, ret);
		return;
	}

	pwm_set_enable(pwm_dev, 0, 0);
}
#endif

static void platform_viewfinder_disable(void)
{
	/* On early boards the viewfinder starts with full backlight
	 * This code is to be able to turn off the viewfinder early.
	 */
	const int VFM_PWR_EN = IMX_GPIO_NR(4, 20);
	gpio_request(VFM_PWR_EN, "VFM_PWR_EN");
	gpio_direction_output(VFM_PWR_EN, false);
}

int board_init(void)
{
	platform_viewfinder_disable();

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_DM_REGULATOR)
	regulators_enable_boot_on(false);
#endif

#ifdef CONFIG_MXC_SPI
	platform_setup_pmic_voltages();
	platform_check_pmic_boot_reason();
#endif

#ifdef CONFIG_DM_I2C
	platform_check_fuelgauge();
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

	if (IS_ENABLED(CONFIG_VIDEO_IPUV3)) {
		struct mipi_dsi_ops ops;

		setup_display();
		if (detect_truly(NULL)) {
			ops.get_lcd_videomode = mipid_st7703_get_lcd_videomode;
			ops.lcd_setup = mipid_st7703_lcd_setup;
		}
		mxc_mipi_dsi_enable(&ops);
	}

	return 0;
}


int board_late_init(void)
{

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EOCO");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

	run_command("mw.l 20e0154 5 1", 0); //set pinmux for GPIO5.0 to gpio pin

	// cooler is temporally disabled by setting the variable 'cooler' to off (any value)
	// cooler is reeanbled by pressing the joystick button during reset cycle (power on)
	// i.e.
	//   - joystick button pressed during uboot start
	//     - and cooler variable is set to any value
	//       - clear cooler variable
	//       - save environment
	//  Thus, during forthcoming boots, the cooler will be enabled

	if (run_command("button press", 0) == 0) {
		if (env_get("cooler")) {
			env_set("cooler", "");
			env_save();
		}
	}

	if (env_get("cooler")) {
		run_command("gpio set GPIO5_0", 0);
	}

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	setup_spinor();
	return 0;
}

static int fpga_power(bool enable)
{
	unsigned char conf_id;

	if (pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id)) {
		printf("Could not read PMIC ID registers\n");
		spi_release_bus(slave);
		return -EIO;
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
	if (conf_id == 0x3b) { //revC
		// BMEM_EN         (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_BMEM_CONT, DA9063_BUCK_EN,
				    enable ? DA9063_BUCK_EN : 0);
		// LDO10_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
	} else { //revD
		// LDO10_EN          (2V5_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO10_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
		// LDO8_EN          (3V15_FPGA)
		pmic_write_bitfield(DA9063_REG_LDO8_CONT, DA9063_LDO_EN,
				    enable ? DA9063_LDO_EN : 0);
	}

	//Enable BMEM_CONT regualtor, 1V1_FPGA, FPGA Core voltage, Only EOCO??
	pmic_write_bitfield(DA9063_REG_BMEM_CONT, DA9063_BUCK_EN,
			    enable ? DA9063_BUCK_EN : 0);
	spi_release_bus(slave);

	return 0;
}

#define GPIO_SPI1_SCLK     IMX_GPIO_NR(5, 22)
#define GPIO_SPI1_MOSI     IMX_GPIO_NR(5, 23)
#define GPIO_SPI1_MISO     IMX_GPIO_NR(5, 24)
#define GPIO_SPI1_CS       IMX_GPIO_NR(5, 28)

/**
 * @brief Fill fpga structure
 *
 * @param fpga
 */
static void eoco_fpga_set_ctrl(struct fpga_ctrl *fpga)
{
#if ! IS_ENABLED(CONFIG_FPGA_ALTERA)
#error "eoco needs to have CONFIG_FPGA_ALTERA set"
#endif

	fpga->pins.config_n = IMX_GPIO_NR(1, 7);
	fpga->pins.status_n = IMX_GPIO_NR(1, 2);
	fpga->pins.done = IMX_GPIO_NR(1, 8);
	fpga->pins.ce = IMX_GPIO_NR(5, 25);
	fpga_set_ops(fpga);
}

static int eoco_fpga_enable_power(struct fpga_ctrl *fpga)
{
	debug("%s\n",  __func__);
	return fpga_power(true);
}

static int eoco_fpga_request_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//Use as cpu spi bus
	ret = gpio_request(GPIO_SPI1_CS, "CS SPI1 0");
	if (ret)
		log_err("%s: gpio request failure %d\n",  __func__, ret);

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	ret = gpio_direction_output(GPIO_SPI1_CS, 1);
	if (ret)
		log_err("%s: gpio dir failure %d\n",  __func__, ret);

	return 0;
}

static int eoco_fpga_release_flash_spi(struct fpga_ctrl *fpga)
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

	if (ret) {
		log_err("%s: gpio failure\n",  __func__);
		return ret;
	}

	return 0;
}

static int do_spi_xfer(int bus, int cs, int freq, int mode, uchar *dout, int len)
{
	struct spi_slave *slave;
	int ret = 0;
	int bitlen = (1 + len) * 8;
	uchar din[SPI_FLASH_MAX_SIZE_BUF];

	if (CONFIG_IS_ENABLED(DM_SPI)) {
		char name[30], *str;
		struct udevice *dev;

		snprintf(name, sizeof(name), "generic_%d:%d", bus, cs);
		str = strdup(name);
		if (!str)
			return -ENOMEM;
		ret = spi_get_bus_and_cs(bus, cs, freq, mode, "spi_generic_drv",
					 str, &dev, &slave);
		if (ret)
			return ret;
	} else {
		slave = spi_setup_slave(bus, cs, freq, mode);
		if (!slave) {
			printf("Invalid device %d:%d\n", bus, cs);
			return -EINVAL;
		}
	}

	ret = spi_claim_bus(slave);
	if (ret)
		goto done;

	ret = spi_xfer(slave, bitlen, dout, din,
		       SPI_XFER_BEGIN | SPI_XFER_END);
	if (!CONFIG_IS_ENABLED(DM_SPI)) {
		/* We don't get an error code in this case */
		if (ret)
			ret = -EIO;
	}

done:
	spi_release_bus(slave);
	if (!CONFIG_IS_ENABLED(DM_SPI))
		spi_free_slave(slave);

	return ret;
}

/**
 * @brief Write a command to the spi flash
 *
 * @param cmd
 * @param dout the data to be written, null if no parameters to cmd
 * @param len length of dout
 * @return int
 */
static int spi_flash_cmd(uchar cmd, uchar *dout, size_t len)
{
	uchar buf[SPI_FLASH_MAX_SIZE_BUF] = {cmd};
	unsigned int bus = 0;
	unsigned int cs = 0;
	unsigned int mode = CONFIG_DEFAULT_SPI_MODE;
	unsigned int freq = 1000000;

	if ((len + 1 >= SPI_FLASH_MAX_SIZE_BUF) || (len > 0 && !dout))
		return -EINVAL;

	if (len > 0)
		memcpy(&buf[1], dout, len);

	return do_spi_xfer(bus, cs, freq, mode, buf, len);
}

/**
 * @brief Set up spi flash according to altera spec
 *
 * @return int negative on error
 */
static int setup_fpga_spi_flash(void)
{
	uchar hold_disable_mask = 0xef;
	int ret = 0;

	//enable_spi_bus(); //take control of spi bus
	ret += spi_flash_cmd(CMD_WRITE_ENABLE, NULL, 0);
	ret += spi_flash_cmd(CMD_WRITE_ENHANCED_VOLATILE_CONF, &hold_disable_mask, 1);
	ret += spi_flash_cmd(CMD_WRITE_ENABLE, NULL, 0);
	ret += spi_flash_cmd(CMD_EN4BYTE_ADDR, NULL, 0);

	return ret;
}

static int eoco_fpga_init_spi_flash(struct fpga_ctrl *fpga)
{
	return setup_fpga_spi_flash();
}

static void fpga_set_board_ops(struct fpga_board_ops *ops)
{
	ops->fpga_request_flash_spi = eoco_fpga_request_flash_spi;
	ops->fpga_release_flash_spi = eoco_fpga_release_flash_spi;
	ops->fpga_enable_power = eoco_fpga_enable_power;
	ops->fpga_init_spi_flash = eoco_fpga_init_spi_flash;
}

void fpga_init_ctrl(struct fpga_ctrl *fpga)
{
	debug("%s:\n", __func__);
	eoco_fpga_set_ctrl(fpga);

	fpga_set_board_ops(&fpga->board_ops);
}
