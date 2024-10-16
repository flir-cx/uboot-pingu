// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <errno.h>
#include <vsprintf.h>
#include <stdio_dev.h>
#include <asm/mach-imx/video.h>
#include <video.h>
#include <video_font.h>
#include <dm/uclass.h>
#include "cmd_kbd.h"
#include "cmd_recoverykey.h"
#include "cmd_kbdsecret.h"
#include "eeprom.h"

#define ESC "\x1b"
#define CSI "\x1b["
#define CLR_LINE CSI "2K"
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static bool has_console = false;

#ifndef CONFIG_DM_VIDEO

static int init_stdio(void) { return -ENODEV; }
static int compute_stdio_dimensions(void) { return -ENODEV; }
static inline void print_display(char *s) {}
static int set_cursor_pos(unsigned int row, unsigned int col) { return 0; }
static int print_recovery_banner(void) { return 0; }
void print_custom_banner(const char *message) { printf("%s\n", message); }

#else

static struct stdio_dev *sdev;
struct stdio_dim {
	unsigned int rows;
	unsigned int cols;
};

static struct stdio_dim sdim;

static inline void print_display(const char *s)
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
	int i;
	struct display_info_t const *dev;

	for (i = 0; i < display_count; i++) {
		dev = displays + i;
		if (displays[i].detect && displays[i].detect(dev))
			break;
	}
	if (i == display_count) {
		printf("Did not find a console video device\n");
		return -ENODEV;
	}

	sdim.rows = displays[i].mode.yres / VIDEO_FONT_HEIGHT;
	sdim.cols = displays[i].mode.xres / VIDEO_FONT_WIDTH;

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
	struct hw_version prodinfo;
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

	snprintf(msg, sizeof(msg), "Product: %s", prodinfo.name);
	if (set_cursor_pos(row + 2, col))
		goto bail_out;
	print_display(msg);
	snprintf(msg, sizeof(msg), " Serial: %d", prodinfo.serial);
	if (set_cursor_pos(row + 3, col))
		goto bail_out;
	print_display(msg);

	return 0;

bail_out:
	print_display(CLR_LINE "\r:..Recovery");
	return 0;
}

void print_custom_banner(const char *message)
{
	unsigned int row;
	unsigned int col;
	unsigned int mlen = strlen(message);

	if (!sdim.rows || !sdim.cols) {
		print_display(message);
		return;
	}

	set_cursor_pos(1, 1);
	print_display(CLR_LINE);
	row = sdim.rows * 2 / 3;
	col = (sdim.cols - mlen) / 2;
	if (set_cursor_pos(row, col))
		print_display(CLR_LINE);
	print_display(message);
}

#endif // CONFIG_DM_VIDEO

/*
 * read_one_key() - Read one key from the keypad
 *
 * @param rc: Returned key label, or 0
 * Return: 0 When nothing pressed after 3s
 *         >0 when keypress detected
 *         <0 on error
 */
static int read_one_key(char *rc)
{
	int numpressed;
	int timeout = 300;
	int key_down = 0;
	char *keybuf;

	if (!rc)
		return -EINVAL;

	*rc = 0;
	while (--timeout) {
		numpressed = read_keys(&keybuf);
		if (numpressed < 0) {
			printf("Failed to read keyboard\n");
			return numpressed;
		}

		if (!key_down && numpressed) {
			*rc = keybuf[0];
			key_down = 1;
		}

		if (key_down && !numpressed)
			break;

		mdelay(10);
	}

	return timeout;
}

static int wait_if_key_already_pressed(void)
{
	int timeout = 500;
	int numpressed;
	char *keybuf;
	bool message_was_written = false;

	while (--timeout) {
		numpressed = read_keys(&keybuf);
		if (numpressed < 0) {
			printf("Failed to read keyboard\n");
			return numpressed;
		}
		if (numpressed == 0) {
			set_cursor_pos(1, 1);
			print_display(CLR_LINE ":");
			return 1;
		}
		if (!message_was_written) {
			print_display("Release pressed keys:");
			message_was_written = true;
		}
		mdelay(10);
	}
	return 0;
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
	char *recovery_string = CONFIG_FLIR_RECOVERY_SEQUENCE;
	char *callback_string = CONFIG_FLIR_CUSTOM_CB_SEQUENCE;
	char rbuf[16];
	int pos = 0;
	int max_string_length;
	int ret = 0;

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
	ret = wait_if_key_already_pressed();
	if (ret == 0)
		return 1;

	max_string_length = MAX(strlen(recovery_string), strlen(callback_string));
	if (max_string_length >= sizeof(rbuf)) {
		log_err("Key sequences must contain less than %d characters\n", sizeof(rbuf));
		max_string_length = sizeof(rbuf) - 1;
	}
	if (max_string_length == 0)
		log_err("Recovery-key sequence is empty. Always boot to recovery\n");

	while (pos < max_string_length) {
		int keypressed = read_one_key(&rbuf[pos]);

		if (keypressed < 0) {
			if (has_console)
				print_display(" *kbd error* ");
			printf("Key polling failed\n");
			break;
		}

		if (!keypressed)
			break;

		if (has_console)
			print_display(".");

		pos++;
	};
	rbuf[pos] = 0;

	if (!strncmp(rbuf, recovery_string, sizeof(rbuf))) {
		if (has_console)
			print_recovery_banner();
		else
			print_display("Recovery boot");
		ret = 0;
	} else if (!strncmp(rbuf, callback_string, sizeof(rbuf))) {
		kbdsecret_custom_callback();
		ret = 1;
	} else {
		print_display("boot");
		printf("Secret didn't match anything\n");
		ret = 1;
	}

	return ret;
}

__weak void kbdsecret_custom_callback(void)
{
	print_custom_banner("Custom function invoked");
}

U_BOOT_CMD(kbd_secret, 1, 1, do_kbd_secret,
	   "",
	   "Returns 0 (true) to shell if success."
);
