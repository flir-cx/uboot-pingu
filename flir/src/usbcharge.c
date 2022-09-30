
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
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <env.h>
#include <i2c.h>
#include <linux/delay.h>
#include <spi.h>
#include <log.h>
#include <dm/uclass.h>
#include "../../../flir/include/da9063.h"
#include "../../../flir/include/da9063_regs.h"

extern struct spi_slave *slave;
u32 get_imx_reset_cause(void);

enum WAKE_EVENTS
{
	USB_CABLE = 0,
	VBUS_POWER,
	ONKEY,
	RTC_TICK,
	RESET,
	ONKEY_LONG_PRESS,
	INVALID_EVENT,
};

enum BOOT_STATES
{
	NORMAL_BOOT = 0,
	LOW_BATTERY,
	NO_BATTERY,
	USB_CHARGE,
};

static struct boot_state
{
	int boot_reason;
	int boot_state;
	unsigned char battery_level;
	bool battery;
	bool usb_cable;

}state = {
	.boot_state = NORMAL_BOOT,
};



//do not boot camera if battery level is below
#define LOW_BATTERY_LEVEL 3

#define charge_state_cmd "fad.power_state=3 systemd.unit=charge.target"
#define BQ27542_I2C_ADDR 0x55
#define BQ27542_REG_STATE_OF_CHARGE 0x2C


static void power_off(bool comparator_enable)
{
	spi_claim_bus(slave);
	printf("Powering off....\n");
	if(comparator_enable) //enable wake up from comparator events
		pmic_write_bitfield(DA9063_REG_ADC_CONT, DA9063_COMP1V2_EN, DA9063_COMP1V2_EN);
	mdelay(100);
 	pmic_write_bitfield(DA9063_REG_CONTROL_C, DA9063_DEBOUNCING_MASK, DA9063_DEBOUNCING_256MS);
	pmic_write_bitfield(DA9063_REG_CONTROL_C, DA9063_AUTO_BOOT, 0);
	pmic_write_bitfield(DA9063_REG_IRQ_MASK_A, DA9063_M_TICK, DA9063_M_TICK);
	pmic_write_bitfield(DA9063_REG_CONTROL_A, DA9063_SYSTEM_EN, 0);

	while(1)
	{

	}
}

int get_boot_reason(void)
{

	unsigned char event_a,event_b,fault_log,status_a;
	int boot_reason = INVALID_EVENT;
  	spi_claim_bus(slave);

	pmic_read_reg(DA9063_REG_STATUS_A,&status_a);

	pmic_read_reg(DA9063_REG_FAULT_LOG,&fault_log);
	pmic_write_reg(DA9063_REG_FAULT_LOG,fault_log);

	pmic_read_reg(DA9063_REG_EVENT_A,&event_a);
	pmic_write_reg(DA9063_REG_EVENT_A,event_a);

	pmic_read_reg(DA9063_REG_EVENT_B,&event_b);
	pmic_write_reg(DA9063_REG_EVENT_B,event_b);
	printf("DA9063: event_a = 0x%x , event_b = 0x%x status_a = 0x%x fault_log = 0x%x\n",event_a,event_b,status_a,fault_log);

	//Long press on-key button
	if(fault_log & DA9063_KEY_RESET)
		boot_reason = ONKEY_LONG_PRESS;
	//Reset event
	else if( (fault_log & (DA9063_NSHUTDOWN | DA9063_TWD_ERROR)) || (get_imx_reset_cause() & 0x10) )
		boot_reason = RESET;
	//On key
	else if(event_a & DA9063_E_NONKEY)
		boot_reason = ONKEY;
	//Usb cable
	else if(event_b & DA9063_E_WAKE)
		boot_reason = USB_CABLE;
	//Battery insertion and Usb cable already attached
	else if( (event_b & DA9063_E_COMP_1V2) && (status_a & DA9063_WAKE) )
		boot_reason = USB_CABLE;
	//Only Usb cable, but prepare camera for battery wake
	else if(status_a & DA9063_WAKE )
		boot_reason = VBUS_POWER;
	//Rtc tick
	else if(event_a & DA9063_E_TICK)
		boot_reason = RTC_TICK;

	spi_release_bus(slave);

	state.boot_reason = boot_reason;
	state.usb_cable = status_a & DA9063_WAKE;
	state.battery   = status_a & DA9063_COMP_1V2;

	return boot_reason;
}

int get_battery_level(void)
{
	int ret;

#if !CONFIG_IS_ENABLED(DM_I2C)
	unsigned char battery_level;
	i2c_set_bus_num(3);

	ret = i2c_read(BQ27542_I2C_ADDR, BQ27542_REG_STATE_OF_CHARGE, 1, &battery_level, 1);
	if(ret<0)
	{
		debug("Battery: missing \n");
		return ret;
	}
	printf("Battery: charge level %d%% \n",battery_level);
	state.battery_level = battery_level;

	return battery_level;

#else // CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *bus, *dev;
	
	state.battery_level = 0;
	ret = uclass_get_device_by_seq(UCLASS_I2C, 3, &bus);
	if (ret != 0)
	{
		debug("uclass_get_device_by_seq() error!\n");
		return ret;
	}

	ret = dm_i2c_probe(bus, BQ27542_I2C_ADDR, 0, &dev);
	if (ret == 0)
	{
		ret = dm_i2c_read(dev, BQ27542_REG_STATE_OF_CHARGE, &state.battery_level, 1);
		if (ret == 0)
		{
			printf("Battery: charge level %d%%\n",state.battery_level);
			return (((int) (state.battery_level)) & 0x000000ff);
		}
		else
		{
			debug("Battery: dm_i2c_read() error! Battery missing or charge level is 0%%. Returning %d\n", ret);
			return ret;
		}
	}
	else
	{
		debug("Battery: dm_i2c_probe() error! Battery missing or charge level is 0%%. Returning %d\n", ret);
		return ret;
	}
#endif // !CONFIG_IS_ENABLED(DM_I2C)
}

void set_boot_logo(void)
{

	switch(state.boot_state)
	{
	case NO_BATTERY:
	case LOW_BATTERY:
		env_set("bootlogo","no_battery.bmp.gz" );
		break;

	case USB_CHARGE:
		env_set("bootlogo","battery_logo.bmp.gz" );
		break;
	case NORMAL_BOOT:
	//	setenv("bootlogo","bootlogo.bmp.gz" );
		break;

	}

}

int usb_charge_setup(void)
{

	int battery_level = get_battery_level();
	int boot_reason = get_boot_reason();
	state.boot_state = NORMAL_BOOT;

	//for cameras without a fuelgauge, we decide if we have a battery by looking at comparator level
	if(battery_level < 0 && state.battery)
	{
		printf("Battery: fuel gauge missing, fakeing a battery \n");
		battery_level = 50;
	}

	switch(boot_reason)
	{
	case USB_CABLE:
		if(battery_level < 0)
			state.boot_state = NO_BATTERY;
		else
			state.boot_state = USB_CHARGE;
		break;

	case RESET:
	case ONKEY:
		if(battery_level < 0)
			state.boot_state = NO_BATTERY;
		else if(battery_level < LOW_BATTERY_LEVEL)
		{
			if (state.usb_cable)
				state.boot_state = USB_CHARGE;
			else
				state.boot_state = LOW_BATTERY;
		}
		break;
	case VBUS_POWER:
		if(battery_level >= 0)
		{
			state.boot_state = USB_CHARGE;
			break;
		}
		printf("Invalid boot event: VBUS_POWER \n");
		power_off(true);
	case ONKEY_LONG_PRESS:
		printf("Invalid boot event: ONKEY_LONG_PRESS \n");
		power_off(false);
		break;
	case INVALID_EVENT:
		printf("Invalid boot event: INVALID_EVENT \n");
		power_off(false);
		break;
	case RTC_TICK:
		printf("Invalid boot event: RTC_TICK \n");
		power_off(false);
		break;
	}


	return 0;
}




static int do_charge_state(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{

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
		power_off(true);
		break;

	case LOW_BATTERY:
		printf("Battery low power\n");
		power_off(false);
		break;

	case USB_CHARGE:
		printf("Camera: charge state \n");
		env_set("charge_state",charge_state_cmd );
		break;

	default:
		printf("Invalid boot state \n");
		break;
	}

	return 0;

}

U_BOOT_CMD(
    chargeState,	2,	0,	do_charge_state,
    "Process charge state, might power off camera",
	" {state} \n"
	"0 - Normal state		-> boot camera into run state\n"
	"1 - Low battery state		-> power off camera\n"
	"2 - No battery			-> power off camera\n"
	"3 - Charge battery		-> boot camera into charge state\n"
);

