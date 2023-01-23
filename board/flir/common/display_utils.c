// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <dm.h>
#include <video.h>
#include <video_console.h>
#include <time.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>

#define DISPLAY_ENABLE		IMX_GPIO_NR(3, 6)

#define DISPLAY_COLOR_GREEN	0x3dbe54
#define DISPLAY_COLOR_YELLOW	0xfff959
#define DISPLAY_COLOR_RED	0xe93b3b

#define DISPLAY_TIMEOUT		20

bool display_enabled = true;
u64  display_timer;

void display_set_text_color(void)
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
	vid_priv->colour_fg = vid_console_color(vid_priv, vid_priv->fg_col_idx);

	/* background color, also mask the bold bit */
	vid_priv->bg_col_idx &= ~0xf;
	vid_priv->bg_col_idx |= 0;
	vid_priv->colour_bg = vid_console_color(vid_priv, vid_priv->bg_col_idx);
}

void display_print_string(char *s)
{
	char buf[64];
	struct udevice *dev_console;
	int xpos, ypos;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	xpos = (80 / 2) - (strlen(s) / 2);
	ypos = 19;

	vidconsole_position_cursor(dev_console, xpos, ypos);
	snprintf(buf, 64, "%s", s);
	vidconsole_put_string(dev_console, buf);

	video_sync(dev_console->parent, true);
}

void display_print_charge_level(int c)
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

	video_sync(dev_console->parent, true);
}

void display_draw_box(int x_start, int y_start, int width, int height, int color, void *framebuffer)
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

	while (height--) {
		for (int w = 0; w < width; w++) {
			*(u32 *)dst = color;
			dst += bpp;
		}
		dst += skip;
	}

	video_sync(dev, true);
}

void display_update_charge(int level)
{
	int color = DISPLAY_COLOR_GREEN;

	if (level < 20)
		color = DISPLAY_COLOR_RED;
	else if (level < 60)
		color = DISPLAY_COLOR_YELLOW;

	display_draw_box(276, 217, level, 45, color, (void *)(gd->fb_base));

	display_print_charge_level(level);
}

void display_timer_reset(void)
{
	display_timer = get_ticks();
}

void display_on(void)
{
	gpio_direction_output(DISPLAY_ENABLE, 1);
	display_timer_reset();
	display_enabled = true;
}

void display_off(void)
{
	if (!display_enabled)
		return;
	gpio_direction_output(DISPLAY_ENABLE, 0);
	display_enabled = false;
}

bool display_is_on(void)
{
	return display_enabled;
}

void display_check_timer(void)
{
	u64 etime = display_timer + CONFIG_SYS_HZ_CLOCK * DISPLAY_TIMEOUT;

	if (get_ticks() > etime)
		display_off();
}
