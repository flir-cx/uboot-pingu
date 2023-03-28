// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <errno.h>

static int do_kbd_secret(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}

U_BOOT_CMD(kbd_secret, 1, 1, do_kbd_secret,
	   "legacy compatibility",
	   "Returns 0 (true)."
);
