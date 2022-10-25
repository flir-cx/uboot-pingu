#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <dm.h>
#include "bootstate.h"
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include "pf1550.h"
#include "lc709203.h"
#include "leds.h"
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define CHARGEAPP_LOOP_DELAY 100000			/* 100ms */
#define CHARGER_TOGGLE_HYSTERESIS 2500000	/* 2.5s */
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
	int cnt = 0;

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

		if(get_onoff_key()) {
			leds_boot();
			exit=1;
		}

		//poweroff camera if usb-cable is removed
		if(!get_usb_cable_state()) {

			/* When the charger is inserted and target is rebooted,
			 * it will say that the cable is out after 2 seconds.
			 * After another 2 seconds, the cable will be replugged.
			 * Because of this we need a bit of hysteresis, so that the
			 * target doesn't end up in a reboot loop */
			cnt++;
			if (cnt * CHARGEAPP_LOOP_DELAY > CHARGER_TOGGLE_HYSTERESIS) {
				power_off();
			}
		} else {
			cnt = 0;
		}
			
		//exit if ctrlc is pressed
		if(ctrlc())
			exit=1;

		soc_last = soc;

		udelay(CHARGEAPP_LOOP_DELAY);
	}
	return 0;
}

U_BOOT_CMD(
    chargeapp,	2,	0,	do_chargeapp,
    "do_chargeapp",
    "do_chargeapp"
);
