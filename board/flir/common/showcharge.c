// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
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
#include <console.h>
#include <command.h>
#include <stdio_dev.h>
#include <dm.h>
#include <video.h>
#include <video_font.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <log.h>
#include "da9063_regs.h"
#include "usbcharge.h"
#include "showcharge.h"

#define ESC "\x1b"
#define CSI "\x1b["

#define LED_PWM_EN_GPIO	IMX_GPIO_NR(3, 6)
#define DISPLAY_TIMEOUT 20

#define COLOR_WHITE_BPP16	0xffff
#define COLOR_GREEN_BPP16	0x2F2D //0x2100 (dark), 0x2780,0x8f8f (light), 0x2F2D (preferred)
#define COLOR_YELLOW_BPP16	0xff00
#define COLOR_RED_BPP16		0xe800 //0xe800, 0x8080
#define COLOR_GREEN_BPP32	0x003dbe54
#define COLOR_YELLOW_BPP32	0x00fff959
#define COLOR_RED_BPP32		0x00e93b3b

#define DISPLAY_ON_TIME_TEST	40000
#define DISPLAY_ON_TIME		4000

//Enums
enum {
	DISPLAY_ON,
	DISPLAY_OFF
} display_state = DISPLAY_ON;
ulong  display_timer;

int columns;
uint32_t last_battery_level;
int start_line, left_margin, width, height;
bool color_test;
uint16_t cmd_line_color;

static void print_display(char *s)
{
	struct stdio_dev *dev = NULL;

	dev = stdio_get_by_name("vidconsole");
	if (!dev)
		return;

	dev->puts(dev, s);
}

static int video_get_win_dimensions(int *rows, int *cols)
{
	int ret;
	struct udevice *udev;

	ret = uclass_get_device_by_seq(UCLASS_VIDEO, 0, &udev);
	if (!ret) {
		*rows = video_get_ysize(udev) / VIDEO_FONT_HEIGHT;
		*cols = video_get_xsize(udev) / VIDEO_FONT_WIDTH;
		columns = *cols;
	} else {
		log_err("Call of 'uclass_get_device_by_seq()' not successful!!");
		*rows = 0;
		*cols = 0;
		columns = 0;
	}

	return ret;
}

static void print_charge(int c)
{
	char buf[10];
	int y, x;

	video_get_win_dimensions(&x, &y);
	y = y / 2;
	x = x / 2 + 6;

	print_display(CSI "l");
	snprintf(buf, 10, CSI "%d;%dH", x, y);
	print_display(buf);

	snprintf(buf, 10, "%d%%", c);
	print_display(buf);
}

static uint16_t get_color(int level)
{
	int color = COLOR_GREEN_BPP16;

	if (color_test)
		return cmd_line_color;

	if (level < 20)
		color = COLOR_RED_BPP16;
	else if (level < 60)
		color = COLOR_YELLOW_BPP16;
	return color;
}

static void turn_on_display(void)
{
	backlight_on(true);
	display_timer = get_timer(0);
	display_state = DISPLAY_ON;
}

static void turn_off_display(void)
{
	if (display_state == DISPLAY_OFF)
		return;

	backlight_on(false);
	display_state = DISPLAY_OFF;
}

static int draw_box(struct udevice *dev, uint16_t color_code)
{
	struct video_priv *priv = dev_get_uclass_priv(dev);
	int ret;

	switch (priv->bpix) {
	case VIDEO_BPP16:
		if (IS_ENABLED(CONFIG_VIDEO_BPP16)) {
			u16 *ppix;
			u16 *end = priv->fb + priv->fb_size;
			int fuel_width = 0;
			// 640 x 2 x 480 = 614400
			// printf("priv->fb_size = %d\n", priv->fb_size);

			int xx, yy;
			// Draw box
			for (yy = 0; yy < height; yy++) {
				ppix = priv->fb + (640 * 2) * (start_line + yy) + (left_margin * 2);
				if (yy == 0 || yy == 1 || yy == 2 ||
					yy == (height - 3) || yy == (height - 2) ||
					yy == (height - 1)) {
					for (xx = 0; xx < width; xx++) {
						*ppix++ = COLOR_WHITE_BPP16;
						if (ppix == end)
							break;
					}
				} else {
					ppix[0] = COLOR_WHITE_BPP16;
					ppix[1] = COLOR_WHITE_BPP16;
					ppix[2] = COLOR_WHITE_BPP16;
					ppix[width - 3] = COLOR_WHITE_BPP16;
					ppix[width - 2] = COLOR_WHITE_BPP16;
					ppix[width - 1] = COLOR_WHITE_BPP16;
					if (yy > (height/2 - 17) && yy < (height/2 + 17)) {
						int ii;
						for (ii = 0; ii < 10; ii++)
							ppix[width + ii] = COLOR_WHITE_BPP16;

					}
				}
			}

			// Fill the box with relevant color
			if (last_battery_level == 100)
				fuel_width = width;
			else
				fuel_width = (last_battery_level * width) / 100;

			if (fuel_width <= 6)
				//fuel_width = 0;
				fuel_width = 1;
			else
				fuel_width -= 6;

			for (yy = 0; yy < (height - 6); yy++) {
				ppix = priv->fb + (640*2)*(start_line + 3 + yy) + ((left_margin + 3) * 2);
				for (xx = 0; xx < (fuel_width); xx++) {
					*ppix++ = color_code;
					if (ppix == end)
						break;
				}
			}
			break;
		}
	case VIDEO_BPP32: // Not implemented
		if (IS_ENABLED(CONFIG_VIDEO_BPP32)) {
			u32 *ppix = priv->fb;
			u32 *end = priv->fb + priv->fb_size;

			while (ppix < end)
				*ppix++ = COLOR_GREEN_BPP32;
			break;
		}
	default:
		memset(priv->fb, COLOR_YELLOW_BPP16, priv->fb_size);
		break;
	}
	ret = video_sync_copy(dev, priv->fb, priv->fb + priv->fb_size);
	if (ret)
		return ret;

	print_charge(last_battery_level);

	return video_sync(dev, false);
}

static int charge_progress(struct udevice *dev, uint16_t *old_color_code)
{
	uint16_t new_color_code = get_color(last_battery_level);

	if (new_color_code != *old_color_code) {
		log_info("new_color_code = 0x%04X, *old_color_code = 0x%04X\n",
			new_color_code, *old_color_code);
		*old_color_code = new_color_code;
	}
	if (draw_box(dev, new_color_code))
		return -1;
	return 0;
}

int do_chargeapp(bool from_autoboot)
{
	uint16_t exit = 0;
	struct udevice *dev;
	uint16_t old_color_code = 0;
	int ret;

	if (from_autoboot) {
		// Set default values
		if (get_gauge_state())
			last_battery_level = FAKE_BATTERY_LEVEL;
		else
			last_battery_level = get_battery_level(false);
		color_test = false;
		columns = 0;
		start_line = 200;
		left_margin = 240;
		width = 150;
		height = 86;
		cmd_line_color = 0x2F2D;
	}

	old_color_code = get_color((last_battery_level > 0 ? last_battery_level : 1));

	// Create video device
	ret = uclass_first_device_err(UCLASS_VIDEO, &dev);
	if (ret)
		return ret;

	ret = video_clear(dev);
	if (ret)
		return ret;

	ret = draw_box(dev, old_color_code);
	if (ret)
		return ret;

	turn_on_display();


	// The main loop ...
	while (!exit) {
		int32_t soc = 0;
		uint8_t event_a, status_a;

		get_pmic_regs(&event_a, &status_a);
		if (get_gauge_state())
			soc = FAKE_BATTERY_LEVEL;
		else
			soc = get_battery_level(false);

		if (soc < 0) {
			log_err("chargeapp: No battery!!\n");
			exit = 1;
			goto cam_power_off;
		}

		if (from_autoboot) {
			// Normal mode
			// Show the battery status only for a short time and
			// then continue with boot of Linux in USB charge mode.
			if (get_timer(display_timer) >= DISPLAY_ON_TIME) {
				exit = 1;
				return 0;
			}
		} else {
			// Test mode
			if (display_state == DISPLAY_ON) {
				if (get_timer(display_timer) >= DISPLAY_ON_TIME_TEST) {
					ret = video_clear(dev);
					if (ret)
						return ret;
					turn_off_display();
				}
			}

			if (soc != last_battery_level) {
				last_battery_level = soc;
				log_info("chargeapp: last_battery_level '%d%%'\n", last_battery_level);
				if (display_state == DISPLAY_ON) {
					ret = charge_progress(dev, &old_color_code);
					if (ret) {
						return ret;
					}
				}
			}

			if (event_a & DA9063_E_NONKEY) {
				if (display_state == DISPLAY_ON) {
					run_command_list("chargeState 3; run mmcbootflir", -1, 0);
					return 0;
				} else {
					turn_on_display();
					old_color_code = get_color(last_battery_level);
					ret = draw_box(dev, old_color_code);
					if (ret)
						return ret;
				}
			}

			//exit if ctrlc is pressed
			if (ctrlc()) {
				ret = video_clear(dev);
				if (ret)
					return ret;

				turn_off_display();
				last_battery_level = 0;
				exit = 1;
			}
		}

		//poweroff camera if usb-cable is removed
		if (!(status_a & DA9063_WAKE)) {
			log_info("chargeapp: USB cable not connected!\n");
			exit = 1;
			goto cam_power_off;
		}
	}

	return 0;

cam_power_off:
	video_clear(dev);
	ret = video_clear(dev);
	if (ret)
		return ret;
	turn_off_display();
	power_off(true);
	return 0;
}

static int do_chargeapp_cmd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	uint16_t old_color_code = 0;

	// Set default values
	color_test = false;
	columns = 0;
	start_line = 200;
	left_margin = 240;
	width = 150;
	height = 86;
	cmd_line_color = 0x2F2D;

	if (get_gauge_state())
		last_battery_level = FAKE_BATTERY_LEVEL;
	else
		last_battery_level = get_battery_level(false);

	// Handle commands
	if (argc == 2) {
		int cmd = simple_strtoul(argv[1], NULL, 10);
		if (cmd == 0) {
			do_chargeapp(true);
			return 0;
		} else if (cmd != 1) {
			return -1;
		}
	} else if (argc == 3) {
		int cmd = simple_strtoul(argv[1], NULL, 10);
		if (cmd == 2) {
			color_test = true;
			cmd_line_color = (uint16_t) simple_strtoul(argv[2], NULL, 16);
		} else
			return -1;
		old_color_code = cmd_line_color;
	} else if (argc == 6) {
		int cmd = simple_strtoul(argv[1], NULL, 10);
		if (cmd == 1) {
			start_line  = (uint16_t) simple_strtoul(argv[2], NULL, 10);
			if (start_line < 0 || start_line > 200)
				return -1;
			left_margin = (uint16_t) simple_strtoul(argv[3], NULL, 10);
			if (left_margin < 0 || left_margin > 300)
				return -1;
			width = (uint16_t) simple_strtoul(argv[4], NULL, 10);
			if (width < 10 || width > 300)
				return -1;
			height = (uint16_t) simple_strtoul(argv[5], NULL, 10);
			if (height < 40 || height > 200)
				return -1;
		} else
			return -1;
	} else
		return -1;

	log_info("chargeapp: start_line '%d', left_margin '%d', width '%d', height '%d'\n",
		start_line, left_margin, width, height);
	log_info("chargeapp: last_battery_level '%d%%'\n", last_battery_level);

	if (do_chargeapp(false))
		log_err("do_chargeapp not successful!\n");

	return 0;
}

U_BOOT_CMD(
    chargeapp,	6,	0,	do_chargeapp_cmd,
	"Battery image handling when USB charge. (might switch off the camera)",
	"cmd  /{[line, left_margin, width, height]}\n"
	"\t   0 -> Aimed only for autoboot.\n"
	"\t   1 {[<line: 0-200> <Left_margin: 0-300> <width: 10-300> <height: 40-200>]} -> Frame buffer test\n"
	"\t   2 {[<color: 0xXXXX]} -> Color test, default dimensions.\n"
);
