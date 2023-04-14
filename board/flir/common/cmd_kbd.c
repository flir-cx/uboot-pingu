// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <env.h>
#include <button.h>
#include <dm/uclass.h>
#include <dm/platdata.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include <dm/device.h>
#include "cmd_kbd.h"

#if !CONFIG_IS_ENABLED(BUTTON_GPIO)
#error "Must configure BUTTON_GPIO"
#endif

static char kbuf[32];

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed, or a negative error
 *
 * Limited to single-char button labels.
 *
 * Return: Number of keys pressed, or <0 on error
 *         Registered keys are stored in the kbuf
 */
int read_keys(char **buf)
{
	struct udevice *dev;
	int pos;

	memset(kbuf, 0, sizeof(kbuf));
	*buf = kbuf;

	uclass_find_first_device(UCLASS_BUTTON, &dev);
	while (dev && (pos < sizeof(kbuf) - 1)) {
		struct button_uc_plat *plat = dev_get_uclass_plat(dev);

		if (plat->label && !device_probe(dev) && device_active(dev) &&
		    button_get_state(dev) == BUTTON_ON)
			kbuf[pos++] = plat->label[0];

		uclass_find_next_device(&dev);
	}
	return pos;
}

static int do_kbd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char *buf;
	int numpressed = read_keys(&buf);

	env_set("keybd", buf);
	return numpressed == 0;
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(kbd, 1, 1, do_kbd,
	   "Tests for keypresses, sets 'keybd' environment variable",
	   "Returns 0 (true) to shell if key is pressed."
);
#endif

U_BOOT_CMD(flir_kbd, 1, 1, do_kbd,
	   "Tests for keypresses, sets 'keybd' environment variable",
	   "Returns 0 (true) to shell if key is pressed."
);
