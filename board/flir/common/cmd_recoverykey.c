// SPDX-License-Identifier: GPL-2.0+
#include "cmd_kbd.h"
#include <command.h>

#define RECOVERY_KEY  "R" // TODO: Move to device tree (ec101)
#define SW_LOAD       "S" // TODO: Move to device tree (ec501)

static int safe_boot;

int flir_get_safe_boot(void)
{
	return safe_boot;
}

static int do_recoverykey(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char *keybuf;
	char *s;

	if (read_keys(&keybuf) < 0)
		return 1;

	s = strstr(keybuf, SW_LOAD);
	if (s) {
		safe_boot = 1;
		return 0;
	}

	s = strstr(keybuf, RECOVERY_KEY);
	return !s;
}

U_BOOT_CMD(recoverykey, 1, 1, do_recoverykey,
	   "Test for recovery key, right keypad press",
	   "Returns 0 (true) to shell if key is pressed."
);
