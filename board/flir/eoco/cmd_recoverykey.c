// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <errno.h>
#include <vsprintf.h>
#include <linux/delay.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_font.h>
#include <dm/uclass.h>

#define ESC "\x1b"
#define CSI "\x1b["
#define CLR_LINE CSI "2K"

static struct stdio_dev *sdev;

int read_key(const char *button)
{
	return run_command(button, 0);
}

/*
 * Poll keyboard for button presses.
 * timeout in ms
 *
 * Return
 */
static int poll_key(const char *button, unsigned int timeout)
{
	int pressed;
	int iterations = timeout / 10;

	do {
		pressed = read_key(button);
		if (!pressed)
			break;
		mdelay(10);
	} while (iterations--);

	return pressed;
}

static int init_stdio(void)
{
	int ret = 0;

	sdev = stdio_get_by_name("vidconsole");
	if (!sdev) {
		printf("No vidconsole found\n");
		ret = -ENODEV;
	}
	return ret;
}

static inline void print_display(char *s)
{
	if (sdev)
		sdev->puts(sdev, s);
}

static int do_recoverykey(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	int ret;
	int cnt = 0;
	bool has_console = false;
	char button_sequence[] = {'E', 'N', 'S', 0};
	char tmpstr[100];

	if (init_stdio())
		printf("Stdio error, proceed without visual feedback\n");
	else
		has_console = true;

	for (i = 0; button_sequence[i]; i++) {
		snprintf(tmpstr, sizeof(tmpstr), "button %c", button_sequence[i]);
		ret = poll_key(tmpstr, i ? 3000 : 100);
		if (ret == 0) {
			if (has_console) {
				switch (i) {
				case 0:
					run_command("printboardinfo", 0);
					print_display(":");
					break;
				case 1:
					print_display(".");
					break;
				case 2:
					print_display("Booting recovery");
					break;
				default:
					// this case "should" never happen...
					print_display("error case!");
					break;
				}
			}
			cnt++;
			mdelay(100);
		} else {
			//wrong key or no key pressed, stop sequence detection
			break;
		}
	}

	return (cnt == 3) ? 0 : 1;
}

U_BOOT_CMD(recoverykey, 1, 1, do_recoverykey,
	   "detect system recovery keypress",
	   "Returns 0 (true) to shell if success."
);
