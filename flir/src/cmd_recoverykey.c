#include "../../../flir/include/cmd_kbd.h"
#include "../../../flir/include/cmd_recoverykey.h"
#include <common.h>
#include <command.h>
#include <asm/gpio.h>
#include <i2c.h>

static int safe_boot;

int flir_get_safe_boot(void)
{
	return safe_boot;
}

static int do_recoverykey(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	read_keys(envvalue);
	char *s = strstr(envvalue,SW_LOAD);
	if(s)
	{
		safe_boot = 1;
		return 0;
	}

	s = strstr(envvalue,RECOVERY_KEY);
	return s == NULL;
}

U_BOOT_CMD(
	recoverykey, 1, 1, do_recoverykey,
	"Test for recovery key, right keypad press",
	"Returns 0 (true) to shell if key is pressed."
);


