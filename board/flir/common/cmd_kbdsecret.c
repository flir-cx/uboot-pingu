// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <linux/delay.h>
#include <errno.h>
#include <vsprintf.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_font.h>
#include <dm/uclass.h>
#include "cmd_kbd.h"
#include "cmd_kbdsecret.h"
#include "cmd_recoverykey.h"
#include "eeprom.h"

#define ESC "\x1b"
#define CSI "\x1b["
#define CLR_LINE CSI "2K"

// TODO: Centralize EEPROM address info
#define EEPROM_BUS_ID (2)
#define MAIN_EEPROM_I2C_ADDR (0xae)
#define MAIN_EEPROM_I2C_OFFS (0x40)

static struct stdio_dev *sdev;
struct stdio_dim {
	unsigned int rows;
	unsigned int cols;
};

static struct stdio_dim sdim;

static inline void print_display(char *s)
{
	if (sdev)
		sdev->puts(sdev, s);
}

static int init_stdio(void)
{
	sdev = stdio_get_by_name("vidconsole");
	if (!sdev) {
		printf("Cannot find 'vidconsole'\n");
		return -ENODEV;
	}
	return 0;
}

static int compute_stdio_dimensions(void)
{
	int ret;
	struct udevice *udev;

	// 'vidconsole' will always have id 0, n.b., see stdio.c
	ret = uclass_get_device_by_seq(UCLASS_VIDEO, 0, &udev);
	if (ret != 0) {
		printf("Did not find a console video device\n");
		return ret;
	}

	sdim.rows = video_get_ysize(udev) / VIDEO_FONT_HEIGHT;
	sdim.cols = video_get_xsize(udev) / VIDEO_FONT_WIDTH;

	return 0;
}

static int set_cursor_pos(unsigned int row, unsigned int col)
{
	char buf[16];

	if (row >= sdim.rows || col >= sdim.cols) {
		printf("Illegal cursor position (%u, %u)\n", row, col);
		return -EINVAL;
	}
	snprintf(buf, sizeof(buf), "%s%u;%uH", CSI, row, col);
	print_display(buf);

	return 0;
}

static int print_recovery_banner(void)
{
	unsigned int row;
	unsigned int col;
	struct eeprom prodinfo = {
		.i2c_bus = EEPROM_BUS_ID,
		.i2c_address = MAIN_EEPROM_I2C_ADDR,
		.i2c_offset = MAIN_EEPROM_I2C_OFFS
	};
	char msg[32] = "Recovery Mode";
	unsigned int mlen = strlen(msg);

	if (!sdim.rows || !sdim.cols)
		goto bail_out;

	set_cursor_pos(1, 1);
	print_display(CLR_LINE);

	if (eeprom_read_product(&prodinfo))
		goto bail_out;

	row = sdim.rows * 2 / 3;
	col = (sdim.cols - mlen) / 2;
	if (set_cursor_pos(row, col))
		goto bail_out;
	print_display(msg);

	snprintf(msg, sizeof(msg), "Product: %s", prodinfo.product_name);
	if (set_cursor_pos(row + 2, col))
		goto bail_out;
	print_display(msg);
	snprintf(msg, sizeof(msg), " Serial: %d", prodinfo.product_serial);
	if (set_cursor_pos(row + 3, col))
		goto bail_out;
	print_display(msg);

	return 0;

bail_out:
	print_display(CLR_LINE "\r:..Recovery");
	return 0;
}

/*
 * Poll keyboard for 'secret' button presses.
 *
 * Return  0 when nothing pressed after 3s timeout.
 *        >0 when key-press detected
 *        <0 on error
 */
static int poll_key(char k)
{
	int numpressed;
	int timeout = 300;
	int key_down = 0;
	char *keybuf;

	while (--timeout) {
		numpressed = read_keys(&keybuf);
		if (numpressed < 0) {
			printf("Failed to read keyboard\n");
			return numpressed;
		}

		if (!key_down && numpressed && strrchr(keybuf, k))
			key_down = 1;

		if (key_down && !numpressed)
			break;
		mdelay(10);
	}

	return timeout;
}

/*
 * Security check for entering recovery mode.
 * Will poll each secret key for 3 seconds.
 * The 'secret' array is a null-terminated string.
 *
 * Return: 0 (true in shell) when secret-key sequence successfully input
 *         1 (false in shell) when timeout or HW error
 */

static int do_kbd_secret(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char *chp = secret;
	bool has_console = false;

	//MSD_LOAD button overrides security check
	if (flir_get_safe_boot())
		return 0;

	if (init_stdio()) {
		printf("Stdio error, proceed without visual feedback\n");
	} else {
		int ret = compute_stdio_dimensions();

		if (ret == 0)
			has_console = true;
	}

	if (has_console) {
		set_cursor_pos(1, 1);
		print_display(CLR_LINE ":");
	}

	while (*chp) {
		int keypressed = poll_key(*chp);

		if (keypressed < 0) {
			if (has_console)
				print_display(" *kbd error* ");
			printf("Key polling failed\n");
			break;
		}

		if (keypressed == 0) {
			printf("Timeout reading kbd secret\n");
			break;
		}

		if (has_console)
			print_display(".");
		chp++;
	}

	if (has_console && *chp)
		print_display("boot");
	else
		print_recovery_banner();

	return '\0' != *chp;
}

U_BOOT_CMD(kbd_secret, 1, 1, do_kbd_secret,
	   "",
	   "Returns 0 (true) to shell if success."
);
