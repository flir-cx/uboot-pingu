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
#include "bootstate.h"
#include "pf1550.h"
#include "lc709203.h"
#include <version.h>
#include "usbdcd.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	                        (PAD_CTL_PUS_UP)
#define SNVS_LPCR_BTN_PRESS_TIME_DISABLE	(0x3<<16)

/* We need to redefine this, because it is set to the wrong address. This is likely because there are multiple imx7ulp revision. */
#undef SNVS_LP_LPCR
#define SNVS_LP_LPCR	                        (0x41070038)

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

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_DM_USB
	setup_usb();
#endif
	init_pf1550_pmic();
	fuelgauge_init();
	usb_charge_detect();
	boot_state_init();

	//Let onoff button long press poweroff the camera
	pmic_goto_core_off(true);
	//onoff button longpress disabled for snvs block, this functionality is handled by pmic
	writel((readl(SNVS_LP_LPCR) | SNVS_LPCR_BTN_PRESS_TIME_DISABLE), SNVS_LP_LPCR);

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
	do_fixup_by_path_string(blob, "/chosen", "u-boot,version", U_BOOT_VERSION_STRING);

	return 0;
}
#endif
