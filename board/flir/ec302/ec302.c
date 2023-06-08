/*
 * Copyright 2018 FLIR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <fdt_support.h>
#include <usb.h>
#include <dm.h>
#include <asm/mach-imx/video.h>
#include <mxsfb.h>
#include <i2c.h>
#include <version.h>

#include "../common/bootstate.h"
#include "../common/lc709203.h"
#include "../common/usbdcd.h"
#include "../common/pf1550.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	                        (PAD_CTL_PUS_UP)
#define SNVS_LPCR_BTN_PRESS_TIME_DISABLE	(0x3<<16)

/* We need to redefine this, because it is set to the wrong address. This is likely because there are multiple imx7ulp revision. */
#undef SNVS_LP_LPCR
#define SNVS_LP_LPCR	                        (0x41070038)

static iomux_cfg_t const vcm_pwr_en_pad[] = {
	MX7ULP_PAD_PTF16__PTF16 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#define VCM_PWR_EN_1V8_PIN "GPIO6_16"

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_cfg_t const lpuart4_pads[] = {
	MX7ULP_PAD_PTC3__LPUART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7ULP_PAD_PTC2__LPUART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	mx7ulp_iomux_setup_multiple_pads(lpuart4_pads,
					 ARRAY_SIZE(lpuart4_pads));
}

#ifdef CONFIG_DM_USB
static void setup_usb(void)
{
}

int board_ehci_usb_phy_mode(struct udevice *dev)
{
	return USB_INIT_HOST;
}
#endif

int usb_charge_detect(void)
{
	int charge_current_mA;

	usbdcd_get_charge_current_mA(&charge_current_mA);
	printf("USB DCD: Setting charge current to %d\n", charge_current_mA);
	set_charging_current(charge_current_mA);
	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

/* 
 * When ec302 is booted, the vcam circuit seeems to interfere
 * with the i2c bus, causing it to pe pulled low and thus making
 * it unusable. To mitigate this, we disable the 1.8V regulator.
 */
static void disable_vcam_regulator(void)
{
	struct gpio_desc vcm_desc;
	int ret;

	mx7ulp_iomux_setup_multiple_pads(vcm_pwr_en_pad, ARRAY_SIZE(vcm_pwr_en_pad));

	ret = dm_gpio_lookup_name(VCM_PWR_EN_1V8_PIN, &vcm_desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, VCM_PWR_EN_1V8_PIN, ret);
		return;
	}

	ret = dm_gpio_request(&vcm_desc, "vcm_1v8_en");
	if (ret) {
		printf("%s request vcm_1v8_en failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_set_dir_flags(&vcm_desc, GPIOD_IS_OUT);
	if (ret) {
		printf("%s set dir flags failed ret = %d\n", __func__, ret);
		return;
	}
}

int board_init(void)
{
	int battery_inserted;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_DM_USB
	setup_usb();
#endif
	init_pf1550_pmic();

	battery_inserted = fuelgauge_check_battery_insertion();
	fuelgauge_init();
	if ((battery_inserted == BATTERY_INSERTED) && !CONFIG_FLIR_MFG)
		power_off();

	usb_charge_detect();
	boot_state_init();

	//Let onoff button long press poweroff the camera
	pmic_goto_core_off(true);
	//onoff button longpress disabled for snvs block, this functionality is handled by pmic
	writel((readl(SNVS_LP_LPCR) | SNVS_LPCR_BTN_PRESS_TIME_DISABLE), SNVS_LP_LPCR);

	disable_vcam_regulator();

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int board_late_init(void)
{

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#if defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, struct bd_info *bd)
{
	int temp[2];

	temp[0] = cpu_to_fdt32(gd->fb_base);
	temp[1] = cpu_to_fdt32(640 * 480 * 3); /* MIPI_DSI_FMT_RGB888 is 3 bytes per pixel */
	do_fixup_by_path(blob, "lcdif", "bootlogo", temp, sizeof(temp), 0);
	do_fixup_by_path_string(blob, "/chosen", "u-boot,version", U_BOOT_VERSION_STRING);

	return 0;
}
#endif
