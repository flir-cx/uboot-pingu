

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <dm.h>
#include "bootstate.h"
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_console.h>
#include "pf1550.h"
#include "lc709203.h"
#include <linux/delay.h>

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

static void set_charge_text_color(void)
{
	struct udevice *dev_console;
	struct video_priv *vid_priv;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	vid_priv = dev_get_uclass_priv(dev_console->parent);

	/* foreground color */
	vid_priv->fg_col_idx &= ~7;
	vid_priv->fg_col_idx |= 15;
	vid_priv->colour_fg = vid_console_color(
			vid_priv, vid_priv->fg_col_idx);

	/* background color, also mask the bold bit */
	vid_priv->bg_col_idx &= ~0xf;
	vid_priv->bg_col_idx |= 0;
	vid_priv->colour_bg = vid_console_color(
			vid_priv, vid_priv->bg_col_idx);
}

void print_charge(int c)
{
	char buf[10];
	struct udevice *dev_console;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	vidconsole_position_cursor(dev_console, 40, 19);
	snprintf(buf, 10, "%d%%", c);
	vidconsole_put_string(dev_console, buf);
}

void draw_box(int x_start, int y_start, int width, int height, int color, void *framebuffer)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device(UCLASS_VIDEO, 0, &dev);
	if (ret) {
		printf("Couldnt get video device!\n");
		return;
	}

	int bpp = 4; /* Bytes per pixel */
	int stride = video_get_xsize(dev) * bpp; /* Bytes per row */
	int skip = stride - width * bpp; /* Bytes to jump */

	char *dst = framebuffer + (x_start * bpp) + (y_start * stride);

	while(height--) {
		for(int w = 0; w < width; w++) {
			*(u32*)dst = color;
			dst += bpp;
		}
		dst += skip;
	}

	video_sync(dev, true);
}

void do_charge_update(int level)
{
		int color = COLOR_GREEN;

		if(level < 20)
			color = COLOR_RED;
		else if (level < 60)
			color = COLOR_YELLOW;

		draw_box(276, 217, level, 45, color, (void*) (gd->fb_base));

		print_charge(level);
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

	set_charge_text_color();

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
