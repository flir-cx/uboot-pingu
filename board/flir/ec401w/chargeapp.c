

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <dm.h>
#include <bootstate.h>
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include <pf1550.h>
#include <lc709203.h>
#include <leds.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

void test_charge_levels(void)
{
	for(int i=0;i<=100;i+=10)
	{
		leds_charge(i);
		mdelay(2000);
	}
}

static int do_chargeapp(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{

	int exit = 0;

	int soc = 0;
	int soc_last = -1;

	//Test for display charge level with leds
	if(argc == 2 && argv[1][0]=='t')
	{
		test_charge_levels();
		return 0;
	}

	while(!exit)
	{
		//if we are within allowed thermal range, enable charging, otherwise disable
		pf1550_thm_ok_toogle_charging();

		//get battery state of charge
		get_battery_state_of_charge(&soc);

		//update battery progressbar
		if (soc != soc_last)
			leds_charge(soc);

		if(get_onoff_key())
		{
			//Turn on camera
			leds_off();
			reboot();
		}

		//poweroff camera if usb-cable is removed
		if(!get_usb_cable_state()){
			power_off();
		}

		//exit if ctrlc is pressed
		if(ctrlc())
			exit=1;

		soc_last = soc;

		udelay(100000);
	}
	return 0;
}

U_BOOT_CMD(
    chargeapp,	2,	0,	do_chargeapp,
    "do_chargeapp",
    "do_chargeapp"
);
