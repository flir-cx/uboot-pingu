// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 FLIR Systems.
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
#include <console.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch-imx/cpu.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch-imx/cpu.h>
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spi.h>
#include <log.h>
#include <splash.h>
#include <dm/uclass.h>
#include "da9063.h"
#include "da9063_regs.h"
#include "usbcharge.h"

extern struct spi_slave *slave;

enum WAKE_EVENTS {
	USB_CABLE = 0,
	VBUS_POWER,
	ONKEY,
	RTC_TICK,
	RESET,
	ONKEY_LONG_PRESS,
	BATT_POWER,
	INVALID_EVENT,
};

const char *wake_event_names[] = {
	"USB_CABLE",
	"VBUS_POWER",
	"ONKEY",
	"RTC_TICK",
	"RESET",
	"ONKEY_LONG_PRESS",
	"BATT_POWER",
	"INVALID_EVENT"
};

const char *boot_state_names[] = {
	"NORMAL_BOOT",
	"LOW_BATTERY",
	"NO_BATTERY",
	"USB_CHARGE"
};

static struct boot_state
{
	unsigned int boot_reason;
	unsigned int boot_state;
	unsigned char battery_level;
	bool battery;
	bool usb_cable;
	bool gauge_missing;

} state = {
	.boot_state = NORMAL_BOOT,
};

//do not boot camera if battery level is below
#define LOW_BATTERY_LEVEL 3

#define charge_state_cmd "fad.power_state=3 systemd.unit=charge.target"
#define BQ27542_I2C_ADDR 0x55
#define BQ27542_REG_STATE_OF_CHARGE 0x2C

static void print_boot_event(void)
{
	if (state.boot_reason < ARRAY_SIZE(wake_event_names))
		log_info("Boot event: %s\n", wake_event_names[state.boot_reason]);
	else
		log_info("Boot event: undefined (code %d)\n", state.boot_reason);
}

static void print_boot_state(void)
{
	if (state.boot_state < ARRAY_SIZE(boot_state_names))
		log_info("Boot state: %s\n", boot_state_names[state.boot_state]);
	else
		log_info("Boot state: undefined (code %d)\n", state.boot_state);
}

void power_off(bool comparator_enable)
{
	spi_claim_bus(slave);
	log_info("Powering off....\n");
	if (comparator_enable) //enable wake up from comparator events
		pmic_write_bitfield(DA9063_REG_ADC_CONT, DA9063_COMP1V2_EN, DA9063_COMP1V2_EN);
	mdelay(100);
	pmic_write_bitfield(DA9063_REG_CONTROL_C, DA9063_DEBOUNCING_MASK, DA9063_DEBOUNCING_256MS);
	pmic_write_bitfield(DA9063_REG_CONTROL_C, DA9063_AUTO_BOOT, 0);
	pmic_write_bitfield(DA9063_REG_IRQ_MASK_A, DA9063_M_TICK, DA9063_M_TICK);
	pmic_write_bitfield(DA9063_REG_CONTROL_A, DA9063_SYSTEM_EN, 0);

	while (1)
		;
}

void get_pmic_regs(unsigned char *event_a, unsigned char *status_a)
{
	spi_claim_bus(slave);
	pmic_read_reg(DA9063_REG_STATUS_A, status_a);
	pmic_read_reg(DA9063_REG_EVENT_A, event_a);
	pmic_write_reg(DA9063_REG_EVENT_A, *event_a);
	spi_release_bus(slave);
}

int get_boot_reason(void)
{
	unsigned char event_a, event_b, fault_log, status_a;
	int boot_reason = INVALID_EVENT;

	spi_claim_bus(slave);

	pmic_read_reg(DA9063_REG_STATUS_A, &status_a);

	pmic_read_reg(DA9063_REG_FAULT_LOG, &fault_log);
	pmic_write_reg(DA9063_REG_FAULT_LOG, fault_log);

	pmic_read_reg(DA9063_REG_EVENT_A, &event_a);
	pmic_write_reg(DA9063_REG_EVENT_A, event_a);

	pmic_read_reg(DA9063_REG_EVENT_B, &event_b);
	pmic_write_reg(DA9063_REG_EVENT_B, event_b);
	log_info("DA9063: event_a = 0x%x , event_b = 0x%x status_a = 0x%x fault_log = 0x%x\n",
		 event_a, event_b, status_a, fault_log);

	//Long press on-key button
	if (fault_log & DA9063_KEY_RESET)
		boot_reason = ONKEY_LONG_PRESS;
	//Reset event
	else if ((fault_log & (DA9063_NSHUTDOWN | DA9063_TWD_ERROR)) ||
		 (get_imx_reset_cause() & 0x10))
		boot_reason = RESET;
	//On key
	else if (event_a & DA9063_E_NONKEY)
		boot_reason = ONKEY;
	//Usb cable insertion when battery already attached
	else if (event_b & DA9063_E_WAKE)
		boot_reason = USB_CABLE;
	//Battery insertion and Usb cable already attached
	else if ((event_b & DA9063_E_COMP_1V2) && (status_a & DA9063_WAKE))
		boot_reason = USB_CABLE;
	//Only Usb cable, but prepare camera for battery wake
	else if (status_a & DA9063_WAKE)
		boot_reason = VBUS_POWER;
	//Rtc tick
	else if (event_a & DA9063_E_TICK)
		boot_reason = RTC_TICK;
	// Only battery insertion
	else if (event_a & DA9063_E_SEQ_RDY)
		boot_reason = BATT_POWER;

	spi_release_bus(slave);

	state.boot_reason = boot_reason;
	state.usb_cable = status_a & DA9063_WAKE;
	state.battery   = status_a & DA9063_COMP_1V2;

	return boot_reason;
}

/*
 * Read fuel gauge cell and set state.battery_level
 * Return charge level, or <0 on error
 */
int get_battery_level(void)
{
	int ret;
#if !CONFIG_IS_ENABLED(DM_I2C)

	i2c_set_bus_num(3);
	ret = i2c_read(BQ27542_I2C_ADDR, BQ27542_REG_STATE_OF_CHARGE, 1, &state.battery_level, 1);
	if (ret < 0)
		debug("Battery: missing\n");
	else
		ret = ((int)state.battery_level) & 0x000000ff;

#else // CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *bus, *dev;

	state.battery_level = 0;
	ret = uclass_get_device_by_seq(UCLASS_I2C, 3, &bus);
	if (ret != 0) {
		debug("uclass_get_device_by_seq() error!\n");
		return ret;
	}

	ret = dm_i2c_probe(bus, BQ27542_I2C_ADDR, 0, &dev);
	if (ret != 0) {
		debug("Battery: dm_i2c_probe() Battery gauge missing? Returning %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, BQ27542_REG_STATE_OF_CHARGE, &state.battery_level, 1);
	if (ret == 0)
		ret = ((int)state.battery_level) & 0x000000ff;
	else
		debug("Battery: dm_i2c_read() Read error. Returning %d\n", ret);

#endif // !CONFIG_IS_ENABLED(DM_I2C)

	return ret;
}

void set_boot_logo(void)
{
	switch (state.boot_state) {
	case NO_BATTERY:
	case LOW_BATTERY:
		env_set("bootlogo", "no_battery.bmp.gz");
		break;

	case USB_CHARGE:
		env_set("bootlogo", "battery_logo.bmp.gz");
		break;

	case NORMAL_BOOT:
		env_set("bootlogo", "bootlogo.bmp.gz");
		break;
	}
}

bool get_gauge_state(void)
{
	return state.gauge_missing;
}

int usb_charge_setup(void)
{
	int battery_level = get_battery_level();
	int boot_reason = get_boot_reason();

	print_boot_event();
	state.boot_state = NORMAL_BOOT;
	state.gauge_missing = false;

	// For cameras without a fuelgauge (probably test equipment),
	// we decide if we have a battery by looking at comparator level
	if (battery_level < 0 && state.battery) {
		battery_level = FAKE_BATTERY_LEVEL;
		log_info("Battery: fuel gauge missing, fakeing a battery\n");
		state.gauge_missing = true;
	}

	if (battery_level >= 0)
		log_info("Battery: charge level %d%%\n", battery_level);

	switch (boot_reason) {
	case USB_CABLE:
		if (battery_level < 0) {
			state.boot_state = NO_BATTERY;
		} else if (battery_level < LOW_BATTERY_LEVEL) {
			state.boot_state = LOW_BATTERY;
		} else if (!state.gauge_missing) {
			state.boot_state = USB_CHARGE;
			if (get_battery_level() == 100) {
				log_info("Battery: Fully charged\n");
				power_off(true);
				return -1;
			}
		} // else normal boot
		break;

	case RESET:
	case ONKEY:
		if (battery_level < 0)
			state.boot_state = NO_BATTERY;
		else if (battery_level < LOW_BATTERY_LEVEL)
			state.boot_state = LOW_BATTERY;
		// else normal boot
		break;

	case VBUS_POWER:
		if (battery_level >= 0 && !state.gauge_missing) {
			state.boot_state = USB_CHARGE;
			if (get_battery_level() == 100) {
				log_info("Battery: Fully charged\n");
				power_off(true);
				return -1;
			}
		}
		// Invalid boot event: VBUS_POWER
		if (!state.battery)
			log_info("Battery: Missing\n");
		power_off(true);
		break;

	case ONKEY_LONG_PRESS:
	case INVALID_EVENT:
	case BATT_POWER:
	case RTC_TICK:
		// Uninteresting or invalid event:
		// Power off and stop listening to comparator
		power_off(false);
		break;
	}

	return 0;
}

static int do_boot_state(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 2)
		state.boot_state = simple_strtoul(argv[1], NULL, 16);

	print_boot_state();

	switch (state.boot_state) {
	case NORMAL_BOOT:
		break;

	case NO_BATTERY:
		log_info("Battery missing\n");
		power_off(true);
		break;

	case LOW_BATTERY:
		log_info("Battery low power\n");
		power_off(false);
		break;

	case USB_CHARGE:
		if (env_set("charge_state", charge_state_cmd))
			log_err("Was not able to set the env var 'charge_state'!");
#ifdef CONFIG_FLIR_COMMAND_SHOWCHARGE
		run_command("chargeapp 0", 0);
		if (!env_get("charge_state")) {
			state.boot_state = NORMAL_BOOT;
			splash_screen_prepare();
			splash_display();
			log_info("New boot state: NORMAL_BOOT\n");
		}
#else
		log_info("This u-boot was not built with CONFIG_FLIR_COMMAND_SHOWCHARGE=y!\n");
#endif
		break;

	default:
		log_err("Invalid boot state\n");
		break;
	}

	return 0;
}

U_BOOT_CMD(chargeState, 2, 0, do_boot_state,
	   "Process charge state, might power off camera",
	   " {state}\n"
	   "0 - Normal state		-> boot camera into run state\n"
	   "1 - Low battery state		-> power off camera\n"
	   "2 - No battery			-> power off camera\n"
	   "3 - Charge battery		-> boot camera into charge state\n"
	);

