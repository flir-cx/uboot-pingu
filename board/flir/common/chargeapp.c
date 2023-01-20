#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <dm.h>
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include <linux/delay.h>

#if defined(CONFIG_TARGET_MX7ULP_EC401W)
#include "led_utils.h"
#elif defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
#include "display_utils.h"
#endif

#include "lc709203.h"
#include "pf1550.h"

DECLARE_GLOBAL_DATA_PTR;

#define CHARGEAPP_LOOP_DELAY 100000			/* 100ms */
#define CHARGER_TOGGLE_HYSTERESIS 2500000	/* 2.5s */

#if defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
static void test_charge_levels(void)
{
	for(int i = 0; i <= 100; i++)
	{
		display_update_charge(i);
		mdelay(400);
	}
}
#elif defined(CONFIG_TARGET_MX7ULP_EC401W)
static void test_charge_levels(void)
{
	for(int i = 0; i <= 100; i += 10)
	{
		leds_charge(i);
		mdelay(2000);
	}
}
#endif

static void update_charge(void)
{
	u16 soc = 0;
	static u16 soc_last = 0xFFFF;

	if (fuelgauge_get_state_of_charge(&soc))
		return;

	if (soc == soc_last)
		return;

#if defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
	display_update_charge(soc);
#elif defined(CONFIG_TARGET_MX7ULP_EC401W)
	leds_charge(soc);
#endif

	soc_last = soc;
}

static bool check_button(void)
{
	if (!get_onoff_key())
		return false;

#if defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
	if (!display_is_on()) {
		display_on();
		return false;
	}
#elif defined(CONFIG_TARGET_MX7ULP_EC401W)
	leds_boot();
#endif

	return true;
}

static void check_cable(void)
{
	static int cnt = 0;

	if (get_usb_cable_state()) {
		cnt = 0;
		return;
	}

	/* When the charger is inserted and target is rebooted,
	 * it will say that the cable is out after 2 seconds.
	 * After another 2 seconds, the cable will be replugged.
	 * Because of this we need a bit of hysteresis, so that the
	 * target doesn't end up in a reboot loop */
	cnt++;
	if ((cnt * CHARGEAPP_LOOP_DELAY) > CHARGER_TOGGLE_HYSTERESIS)
		power_off();
}

static int do_chargeapp(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
#if defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
	display_timer_reset();
	display_set_text_color();
#endif

	//Test for drawing  charge progess bar on screen
	if(argc == 2 && argv[1][0]=='t')
	{
		test_charge_levels();
		return 0;
	}

	while(true)
	{
		/* Toggle charging if we are inside/outside temp range */
		pf1550_thm_ok_toogle_charging();

		/* Update battery indicators */
		update_charge();

		/* Boot if button is pressed */
		if (check_button())
			break;

		/* Power down if cable is removed */
		check_cable();

		/* Boot if ctrlc is pressed */
		if (ctrlc())
			break;

#if defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
		/* Turn off screen when timer expires */
		display_check_timer();
#endif

		udelay(CHARGEAPP_LOOP_DELAY);
	}

	return 0;
}

U_BOOT_CMD(
    chargeapp,	2,	0,	do_chargeapp,
    "do_chargeapp",
    "do_chargeapp"
);
