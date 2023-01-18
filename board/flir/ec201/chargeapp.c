

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <dm.h>
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_console.h>
#include <linux/delay.h>
#include "bootstate.h"
#include "display_utils.h"

#include "../common/lc709203.h"
#include "../common/pf1550.h"

DECLARE_GLOBAL_DATA_PTR;

#define CHARGEAPP_LOOP_DELAY 100000			/* 100ms */

#define ESC "\x1b"
#define CSI "\x1b["

#define LED_PWM_EN_GPIO	IMX_GPIO_NR(3, 6)

//Configs
#define COLOR_GREEN 	0x3dbe54
#define COLOR_YELLOW 	0xfff959
#define COLOR_RED 		0xe93b3b
#define DISPLAY_TIMEOUT 20

//Enums
enum {
	DISPLAY_ON,
	DISPLAY_OFF
}
display_state = DISPLAY_ON;
uint64_t  display_timer;

void do_charge_update(int level)
{
		int color = COLOR_GREEN;

		if(level < 20)
			color = COLOR_RED;
		else if (level < 60)
			color = COLOR_YELLOW;

		display_draw_box(276, 217, level, 45, color, (void*) (gd->fb_base));

		display_print_charge_level(level);
}

void turn_on_display(void)
{
	gpio_direction_output(LED_PWM_EN_GPIO, 1);
	display_timer = get_ticks();
	display_state = DISPLAY_ON;
}


void turn_off_display(void)
{
	if(display_state == DISPLAY_OFF)
		return;
	gpio_direction_output(LED_PWM_EN_GPIO, 0);
	display_state = DISPLAY_OFF;
}

void display_off_timer(void)
{
	uint64_t etime = display_timer + CONFIG_SYS_HZ_CLOCK * DISPLAY_TIMEOUT;

	if(get_ticks() > etime)
		turn_off_display();
}


void test_charge_levels(void)
{
	for(int i=0;i<=100;i++)
	{
		do_charge_update(i);
		mdelay(400);
	}
}

static int do_chargeapp(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{

	int exit = 0;
	display_timer = get_ticks();

	display_set_text_color();

	//Test for drawing  charge progess bar on screen
	if(argc == 2 && argv[1][0]=='t')
	{
		test_charge_levels();
		return 0;
	}

	while(!exit)
	{
		int soc=0;

		//if we are within allowed thermal range, enable charging, otherwise disable
		pf1550_thm_ok_toogle_charging();

		//get battery state of charge
		get_battery_state_of_charge(&soc);
		//update battery progressbar
		do_charge_update(soc);

		if(get_onoff_key())
		{
			if(display_state == DISPLAY_OFF)
				turn_on_display();
			else 		//turn on camera if onoff key is pressed
			{
				turn_off_display();
				reboot();
			}

		}

		//poweroff camera if usb-cable is removed
		if(!get_usb_cable_state()){
			fuelgauge_sleep();
			power_off();
		}

		//exit if ctrlc is pressed
		if(ctrlc())
			exit=1;

		//turn off screen after 1min
		display_off_timer();

		udelay(CHARGEAPP_LOOP_DELAY);
	}
	return 0;
}

U_BOOT_CMD(
    chargeapp,	2,	0,	do_chargeapp,
    "do_chargeapp",
    "do_chargeapp"
);
