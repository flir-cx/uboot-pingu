// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <linux/delay.h>
#include <errno.h>
#include <vsprintf.h>
#include <stdio_dev.h>
#include <asm/mach-imx/video.h>
#include <video_font.h>
#include "../include/cmd_kbd.h"
#include "../include/cmd_kbdsecret.h"
#include "../include/cmd_recoverykey.h"
#include "../include/eeprom.h"

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
	sdev->puts(sdev, s);
}

static int init_stdio(void)
{
	sdev = stdio_get_by_name("vidconsole");
	cond_log_return(!sdev, -ENODEV, "Cannot find 'vidconsole'\n");
	return 0;
}

static int compute_stdio_dimensions(void)
{
	int i;
	struct display_info_t const *pinfo;
	const char *pname = env_get("panel");

	cond_log_return(!pname, -ENODEV, "Missing 'panel' in environment\n");
	for (i = 0; i < display_count; i++)
		if (!strcmp(pname, displays[i].mode.name)) {
			pinfo = &displays[i];
			break;
		}
	cond_log_return(!pinfo, -ENODEV, "No panel '%s' is defined\n", pname);
	cond_log_return(!pinfo->mode.xres || !pinfo->mode.yres, -EINVAL,
			"Illegal panel dim: (%d, %d)\n", pinfo->mode.xres, pinfo->mode.yres);

	sdim.rows = pinfo->mode.yres / VIDEO_FONT_HEIGHT;
	sdim.cols = pinfo->mode.xres / VIDEO_FONT_WIDTH;

	return 0;
}

static int set_cursor_pos(unsigned int row, unsigned int col)
{
	char buf[16];

	cond_log_return(row >= sdim.rows || col >= sdim.cols, -EINVAL,
			"Illegal cursor position (%u, %u)\n", row, col);
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
	set_cursor_pos(1, 1);
	print_display("Recovery");
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
		cond_log_return(numpressed < 0, numpressed,
				"Failed to read keyboard\n");

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
	int ret;
	char *chp = secret;

	//MSD_LOAD button overrides security check
	if (flir_get_safe_boot())
		return 0;

	ret = init_stdio();
	return_on_status(ret, "fb-stdio error\n");

	ret = compute_stdio_dimensions();
	return_on_status(ret, "fb-stdio error\n");

	// Clear line and indicate ready-for-secret
	set_cursor_pos(1, 1);
	print_display(CLR_LINE ":");

	while (*chp) {
		int result = poll_key(*chp);

		if (result > 0) {
			print_display(".");
		} else if (result < 0) {
			print_display(" *kbd error* ");
			printf("Key polling failed\n");
			break;
		} else {
			printf("Timeout reading kbd secret\n");
			break;
		}
		chp++;
	}

	if (*chp)
		print_display("boot");
	else
		print_recovery_banner();

	return '\0' != *chp;
}

U_BOOT_CMD(kbd_secret, 1, 1, do_kbd_secret,
	   "",
	   "Returns 0 (true) to shell if success."
);
