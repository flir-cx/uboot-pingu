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

/*
 * Overwrite default implementation of i2c_get_chip_for_busnum.
 * Our eeprom doesn't handle probing of the chip well. This is probably
 * due to the chip not being powered fully when the command is issued.
 * If we continuously poll the probing function, the chip starts responding
 * after some time. Usually after 0.5-3 seconds, but removing the probing
 * from i2c_get_chip_for_busnum makes the chip respond on first command.
 */
int i2c_get_chip_for_busnum_flir(int busnum, int chip_addr, uint offset_len,
			    struct udevice **devp())
{
	struct udevice *bus;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, busnum, &bus);
	if (ret) {
		printf("Cannot find I2C bus %d\n", busnum);
		return ret;
	}

	ret = i2c_get_chip(bus, chip_addr, offset_len, devp);
	if (ret) {
		printf("Cannot find I2C chip %02x on bus %d\n", chip_addr,
		      busnum);
		return ret;
	}

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
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
