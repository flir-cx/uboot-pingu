/*
 * Copyright (C) 2019 FLIR Systems.
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
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <dm.h>
#include <i2c.h>
#include <splash.h>
#include <command.h>
#include <linux/delay.h>

#include "../common/lc709203.h"
#include "../common/pf1550.h"

DECLARE_GLOBAL_DATA_PTR;

enum WAKE_EVENTS
{
	USB_CABLE = 0,
	VBUS_POWER,
	ONKEY,
	RESET,
	ONKEY_LONG_PRESS,
	INVALID_EVENT,
};

enum BOOT_STATES
{
	INVALID_STATE = 0,
	NORMAL_BOOT = 1,
	LOW_BATTERY = 2,
	NO_BATTERY = 3,
	USB_CHARGE = 4,
	NUM_BOOT_STATES = 5,
};

static struct boot_state
{
	u8 wake_event;
	u8 boot_state;
	u8 force_boot_state;
	u16 battery_mV;
	bool battery;
	bool usb_cable;

}state = {
	.wake_event = ONKEY,
	.boot_state = NORMAL_BOOT,
	.force_boot_state = 0,
	.battery_mV = 4000,
	.usb_cable = false,
	.battery = false,
};

#define LOW_BATTERY_mV 3300

//Pmic registers defines
//Special register in pmic which we can use to force boot state between reboots
#define FORCE_BOOT_STATE PF1550_CHARG_REG_LED_PWM

static u8 get_force_boot_state(struct udevice *dev)
{
	u8 force_boot_state, buf;

	//read hidden state register
	dm_i2c_read(dev, FORCE_BOOT_STATE, &force_boot_state, 1);
	buf = 0;
	dm_i2c_write(dev, FORCE_BOOT_STATE, &buf, 1);

	printf("Force Boot State 0x%x... ", force_boot_state);
	if(force_boot_state > INVALID_STATE && force_boot_state < NUM_BOOT_STATES){

		switch(force_boot_state)
		{
		case NORMAL_BOOT:
			printf("%s\n", "NORMAL_BOOT");
			break;
		case LOW_BATTERY:
			printf("%s\n", "LOW_BATTERY");
			break;
		case NO_BATTERY:
			printf("%s\n", "NO_BATTERY");
			break;
		case USB_CHARGE:
			printf("%s\n", "USB_CHARGE");
			break;
		default:
			printf("%s\n", "UNKNOWN");
			break;
		}
	}else{
		printf("%s\n", "NONE");
		force_boot_state = INVALID_STATE;
	}
	return force_boot_state;
}

static u8 get_wake_event(struct udevice *dev)
{
	u8 chg_int, wake_event;

	//We have no way to detect onoff button events from poweroff or reboot.
	//We use this as the default event
	wake_event = RESET;

	/* Check if a usb cable inserted interrupt has occured. */
	dm_i2c_read(dev, PF1550_CHARG_REG_CHG_INT, &chg_int, 1);
	dm_i2c_write(dev, PF1550_CHARG_REG_CHG_INT, &chg_int, 1);

	printf("USB cable inserted interrupt (CHG_INT 0x%x)... ", chg_int);
	if(chg_int & CHARG_IRQ_VBUSI ){
		printf("YES\n");
		wake_event = USB_CABLE;
	}else{
		printf("NO\n");
	}

	return wake_event;
}


int get_battery_voltage(int *voltage)
{
	//Read battery voltage
	struct udevice *dev;
	u8 buf[4];
	int ret;

	ret = i2c_get_chip_for_busnum(5, 0xb, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, 0x9, buf, 2);
	if(!ret)
		*voltage = *(u16*)buf;

	return ret;
}

int get_battery_state_of_charge(int *soc)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;

	ret = i2c_get_chip_for_busnum(5, 0xb, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, 0xd, buf, 2);
	if(!ret)
		*soc = *(u16*)buf;

	return ret;
}

void set_boot_logo(void)
{
	switch(state.boot_state)
	{
	case NO_BATTERY:
	case LOW_BATTERY:
		env_set("bootlogo", "no_battery.bmp.gz");
		break;
	case USB_CHARGE:
		env_set("bootlogo", "battery_logo.bmp.gz");
		break;
	case NORMAL_BOOT:
		break;
	}
}

int boot_state_init(void)
{
	struct udevice *dev;
	int ret;
	u8 chg_int_ok, pwrctrl3;

	ret = i2c_get_chip_for_busnum(5, 0x8, 1, &dev);
	if (ret) {
		printf("Cannot find pmic: %d\n", ret);
		return ret;
	}

	state.force_boot_state = get_force_boot_state(dev);
	if(state.force_boot_state != INVALID_STATE)
		return 0;

	state.wake_event = get_wake_event(dev);

	dm_i2c_read(dev, PF1550_PMIC_REG_PWRCTRL3, &pwrctrl3, 1);
	printf("pwrctrl3=0x%x\n", pwrctrl3);

	/* Check if the usb cable is inserted at all. This checks if
	the cable was already inserted at the time of boot. In this
	case cable inserted interrupt would NOT have been
	received. */
	dm_i2c_read(dev, PF1550_CHARG_REG_CHG_INT_OK, &chg_int_ok, 1);

	printf("USB cable connected (CHG_INT_OK 0x%x)... ", chg_int_ok);
	if(chg_int_ok & PF1550_CHG_INT_OK_VBUS_OK ){
		printf("YES\n");
		state.usb_cable = true;
	}else{
		printf("NO\n");
	}

	return 0;
}

int splash_screen_prepare(void)
{
	char *env_loadsplash;

	set_boot_logo();

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

static int do_boot_state(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	if(state.force_boot_state != INVALID_STATE){
		state.boot_state = state.force_boot_state;
	}else{
		switch(state.wake_event)
		{
		case USB_CABLE:
			state.boot_state = USB_CHARGE;
			break;
		case RESET:
		case ONKEY:
			get_battery_voltage((int *)&state.battery_mV);
			printf("Battery voltage mV=%d... ",state.battery_mV);
			if(state.battery_mV < LOW_BATTERY_mV){
				printf("LOW\n");
				state.boot_state = LOW_BATTERY;
			}else{
				printf("OK\n");
				state.boot_state = NORMAL_BOOT;
			}
			break;
		default:
			printf("Invalid boot event: INVALID_EVENT \n");
			fuelgauge_sleep();
			power_off();
			break;
		}
	}

	/* State specified from u-boot prompt has priority over
	   previously set boot_state. */
	if(argc == 2)
	{
		state.boot_state = simple_strtoul(argv[1], NULL, 16);
		printf("Custom boot state=%d \n",state.boot_state);
	}

	switch(state.boot_state)
	{
	case NORMAL_BOOT:
		break;
	case NO_BATTERY:
		printf("Battery missing\n");
	case LOW_BATTERY:
		set_boot_logo();
		splash_display();

		/* Give user a chance to see splash. */
		udelay(2000000);

		fuelgauge_sleep();
		power_off();
		break;
	case USB_CHARGE:
		printf("Camera: charge state \n");
		set_boot_logo();
		splash_display();
		run_command("chargeapp",0);
		break;
	default:
		printf("Invalid boot state\n");
		break;
	}
	return 0;
}

U_BOOT_CMD(
	bootstate,	2,	0,	do_boot_state,
	"Process boot state, might power off camera",
	" {state} \n"
	"1 - Normal state		-> boot camera into run state\n"
	"2 - Low battery state		-> power off camera\n"
	"3 - No battery			-> power off camera\n"
	"4 - Charge battery		-> boot camera into charge state\n"
);
