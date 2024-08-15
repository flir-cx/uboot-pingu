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
#include <asm/mach-imx/video.h>
#include <video.h>
#include <video_font.h>
#include <cpu_func.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <log.h>
#include "da9063_regs.h"
#include "usbcharge.h"
#include "showcharge.h"

DECLARE_GLOBAL_DATA_PTR;

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
#define DISPLAY_ON_TIME		2000

//Enums
enum {
	DISPLAY_ON,
	DISPLAY_OFF
} display_state = DISPLAY_ON;
ulong  display_timer;

enum chargeapp_arg {
	MODE_AUTOBOOT = 0,
	MODE_FB_TEST,
	MODE_COLOR_TEST
};

struct panel_data {
	void *fb;
	u32 fb_size;
	u16 bpp;
	u32 xres;
	u32 yres;
	u32 rows;
	u32 cols;
} panel;

static enum chargeapp_arg chargeapp_mode;
static int columns;
static u32 last_battery_level;
static int start_line, left_margin, width, height;
static bool color_test;
static u16 cmd_line_color;

void __weak backlight_on(bool on) {}

/**
 * Extract video configuration, using the display array.
 * If the display is a PANEL instead of a VIDEO device,
 * it is not possible to go via the video uclass.
 */
static int display_read_config(void)
{
	int ret = 0;
	int i;
	struct display_info_t const *dev;

	for (i = 0; i < display_count; i++) {
		dev = displays + i;
		if (displays[i].detect && displays[i].detect(dev))
			break;
	}
	if (i == display_count) {
		log_err("Failed to find display device\n");
		ret = -ENODEV;
	} else {
		panel.fb = (u16 *)gd->fb_base;
		panel.bpp = VIDEO_BPP16; // The only one implemented at the moment
		panel.xres = displays[i].mode.xres;
		panel.yres = displays[i].mode.yres;
		panel.fb_size = 2 * panel.xres * panel.yres;
		panel.rows = displays[i].mode.yres / VIDEO_FONT_HEIGHT;
		panel.cols = displays[i].mode.xres / VIDEO_FONT_WIDTH;
	}

	return ret;
}

static inline void display_sync(void)
{
	flush_dcache_range((ulong)panel.fb,
			   ALIGN((ulong)panel.fb + panel.fb_size,
				 CONFIG_SYS_CACHELINE_SIZE));
}

static int display_clear(void)
{
	memset(panel.fb, 0, panel.fb_size);
	display_sync();
	return 0;
}

static void print_display(char *s)
{
	struct stdio_dev *dev = NULL;

	dev = stdio_get_by_name("vidconsole");
	if (!dev)
		return;

	dev->puts(dev, s);
}

static void print_charge(int c)
{
	char buf[10];
	int row, col;

	row = panel.rows / 2 + 6;
	col = panel.cols / 2 - 2;

	print_display(CSI "l");
	snprintf(buf, 10, CSI "%d;%dH", row, col); // cursorpos(v,h)
	print_display(buf);

	snprintf(buf, 10, "%3d%%", c);
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

static int draw_box(uint16_t color_code)
{
	switch (panel.bpp) {
	case VIDEO_BPP16:
		if (IS_ENABLED(CONFIG_VIDEO_BPP16)) {
			u16 *ppix;
			u16 *end = panel.fb + panel.fb_size;
			int fuel_width = 0;
			int stride = panel.xres * VNBYTES(panel.bpp);

			int xx, yy;
			// Draw box
			for (yy = 0; yy < height; yy++) {
				ppix = panel.fb + stride * (start_line + yy) + (left_margin * 2);
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
					if (yy > (height / 2 - 17) && yy < (height / 2 + 17)) {
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
				ppix = panel.fb + stride * (start_line + 3 + yy) +
					((left_margin + 3) * 2);
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
			u32 *ppix = panel.fb;
			u32 *end = panel.fb + panel.fb_size;

			while (ppix < end)
				*ppix++ = COLOR_GREEN_BPP32;
			break;
		}
	default:
		memset(panel.fb, COLOR_YELLOW_BPP16, panel.fb_size);
		break;
	}

	print_charge(last_battery_level);

	display_sync();
	return 0;
}

static int charge_progress(uint16_t *old_color_code)
{
	u16 new_color_code = get_color(last_battery_level);

	if (new_color_code != *old_color_code) {
		log_info("new_color_code = 0x%04X, *old_color_code = 0x%04X\n",
			 new_color_code, *old_color_code);
		*old_color_code = new_color_code;
	}
	if (draw_box(new_color_code))
		return -1;
	return 0;
}

static int do_chargeapp(void)
{
	u16 exit = 0;
	u16 old_color_code = 0;
	int ret = 0;

	old_color_code = get_color((last_battery_level > 0 ? last_battery_level : 1));

	ret = display_clear();
	if (ret)
		return ret;

	ret = draw_box(old_color_code);
	if (ret)
		return ret;

	turn_on_display();

	// The main loop ...
	while (!exit) {
		int soc = 0;
		u8 event_a, status_a;

		get_pmic_regs(&event_a, &status_a);
		if (get_gauge_state())
			soc = FAKE_BATTERY_LEVEL;
		else
			soc = get_battery_level();

		if (soc < 0) {
			log_err("chargeapp: No battery\n");
			goto cam_power_off;
		}

		if (chargeapp_mode == MODE_AUTOBOOT) {
			// Normal mode
			// Show the battery status only for a short time and
			// then continue with boot of Linux in USB charge mode.
			if (get_timer(display_timer) >= DISPLAY_ON_TIME)
				exit = 1;
			// Check power key
			if (status_a & DA9063_NONKEY) {
				if (env_set("charge_state", ""))
					log_err("Failed to clear env 'charge_state'");
				exit = 1;
			}
		} else {
			// Test mode
			if (display_state == DISPLAY_ON) {
				if (get_timer(display_timer) >= DISPLAY_ON_TIME_TEST) {
					ret = display_clear();
					if (ret)
						return ret;
					turn_off_display();
				}
			}

			if (soc != last_battery_level) {
				last_battery_level = soc;
				log_info("chargeapp: last_battery_level '%d%%'\n",
					 last_battery_level);
				if (display_state == DISPLAY_ON) {
					ret = charge_progress(&old_color_code);
					if (ret)
						return ret;
				}
			}

			if (event_a & DA9063_E_NONKEY) {
				if (display_state == DISPLAY_ON) {
					run_command_list("chargeState 3; run mmcbootflir", -1, 0);
					exit = 1;
				} else {
					turn_on_display();
					old_color_code = get_color(last_battery_level);
					ret = draw_box(old_color_code);
					if (ret)
						return ret;
				}
			}

			//exit if ctrlc is pressed
			if (ctrlc()) {
				ret = display_clear();
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

	return ret;

cam_power_off:
	display_clear();
	turn_off_display();
	power_off(true);

	return ret;
}

static int do_chargeapp_cmd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = -1;
	u16 old_color_code = 0;

	display_read_config();

	// Set default values
	color_test = false;
	columns = 0;
	start_line = panel.yres * 5 / 12;
	left_margin = panel.xres * 3 / 8;
	width = panel.xres * 15 / 64;
	height = panel.yres * 86 / 480;
	cmd_line_color = 0x2F2D;

	if (get_gauge_state())
		last_battery_level = FAKE_BATTERY_LEVEL;
	else
		last_battery_level = get_battery_level();

	if (argc >= 2)
		chargeapp_mode = simple_strtoul(argv[1], NULL, 10);
	else
		chargeapp_mode = MODE_AUTOBOOT;

	// Ignore extra arguments, n.b.
	switch (chargeapp_mode) {
	case MODE_AUTOBOOT:
		do_chargeapp();
		return 0;

	case MODE_FB_TEST:
		if (argc < 6)
			break;
		start_line  = (u16)simple_strtoul(argv[2], NULL, 10);
		if (start_line < 0 || start_line > 200)
			break;
		left_margin = (u16)simple_strtoul(argv[3], NULL, 10);
		if (left_margin < 0 || left_margin > 300)
			break;
		width = (u16)simple_strtoul(argv[4], NULL, 10);
		if (width < 10 || width > 300)
			break;
		height = (u16)simple_strtoul(argv[5], NULL, 10);
		if (height < 40 || height > 200)
			break;
		ret = 0;
		break;

	case MODE_COLOR_TEST:
		if (argc < 3)
			break;
		color_test = true;
		cmd_line_color = (u16)simple_strtoul(argv[2], NULL, 16);
		old_color_code = cmd_line_color;
		ret = 0;
		break;
	}

	if (!ret) {
		log_info("chargeapp: start_line '%d', left_margin '%d', width '%d', height '%d'\n",
			 start_line, left_margin, width, height);
		log_info("chargeapp: last_battery_level '%d%%'\n", last_battery_level);
		if (do_chargeapp())
			log_err("do_chargeapp not successful!\n");
	}

	return ret;
}

U_BOOT_CMD(chargeapp, 6, 0, do_chargeapp_cmd,
	   "Battery image handling when USB charge. (might switch off the camera)",
	   "cmd  /{[line, left_margin, width, height]}\n"
	   "\t   0 -> Aimed only for autoboot.\n"
	   "\t   1 {[<line: 0-200> <Left_margin: 0-300> <width: 10-300> <height: 40-200>]} -> Frame buffer test\n"
	   "\t   2 {[<color: 0xXXXX]} -> Color test, default dimensions.\n"
	);
