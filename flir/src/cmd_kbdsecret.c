// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <linux/delay.h>
#include <stdio_dev.h>
#include "../../../flir/include/cmd_kbd.h"
#include "../../../flir/include/cmd_kbdsecret.h"

#define ESC "\x1b"
#define CSI "\x1b["

void print_display(char *s)
{
	struct stdio_dev *dev = NULL;

	dev = stdio_get_by_name("vidconsole");
	if (!dev)
		return;

	dev->puts(dev, s);
}

/*
 * Poll keyboard for 'secret' button presses.
 *
 * Return  0 when nothing pressed after 3s timeout.
 *        >0 when key-press detected
 *        <0 on error
 */
int poll_key(char k)
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
	char *chp = secret;

	//MSD_LOAD button overrides security check
	if (flir_get_safe_boot())
		return 0;

	// Clear line and indicate ready-for-secret
	print_display(CSI "2K");
	print_display("\r");
	print_display(":");

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
		print_display("recovery");

	return '\0' != *chp;
}

U_BOOT_CMD(
	kbd_secret, 1, 1, do_kbd_secret,
	"",
	"Returns 0 (true) to shell if success."
);
