// SPDX-License-Identifier: GPL-2.0+
#include "cmd_kbd.h"
#include <command.h>

static int safe_boot;
static const char *safe_key = CONFIG_FLIR_SAFEBOOT_KEY_VALUE "";
static const char *rec_key = CONFIG_FLIR_RECOVERYKEY_VALUE "";

int flir_get_safe_boot(void)
{
	return safe_boot;
}

static int do_recoverykey(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char *keybuf;
	char *s = NULL;

	if (read_keys(&keybuf) < 0)
		return 1;

	if (*safe_key && strstr(keybuf, safe_key)) {
		safe_boot = 1;
		return 0;
	}

	if (*rec_key)
		s = strstr(keybuf, rec_key);
	else
		log_warning("No recovery-key value is defined\n");

	return !s;
}

U_BOOT_CMD(recoverykey, 1, 1, do_recoverykey,
	   "Test for recovery key, right keypad press",
	   "Returns 0 (true) to shell if key is pressed."
);
