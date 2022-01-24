#include "../../../flir/include/cmd_kbdsecret.h"
#include <common.h>
#include <command.h>
#include <linux/delay.h>
#include <stdio_dev.h>
#include "../../../flir/include/cmd_kbd.h"
#include "../../../flir/include/cmd_recoverykey.h"


#define ESC "\x1b"
#define CSI "\x1b["

void print_display(char *s)
{
	struct stdio_dev *dev = NULL;
	dev = stdio_get_by_name("vga");
	if(!dev)
		return;

	dev->puts(dev,s);
}


int poll_key(char k)
{
	int numpressed,timeout = 300;
	int key_down=0;
	char envvalue[ARRAY_SIZE(buttons)+1];

	while(--timeout)
	{
		numpressed = read_keys(envvalue);
		//check for pressed key
		if(numpressed == 1 && strrchr(envvalue,k)!=0)
			key_down =1;

		//check for release key
		if(key_down && !numpressed)
			break;
		mdelay(10);
	}
	if(timeout)
		return 0;

	return -1;
}



/** Security check for entering recovery mode
 * Will poll each secret keys for 3 seconds.
 * If all keys sucessfully pressed in succession, return success (0)
 *
 */

static int do_kbd_secret(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	//MSD_LOAD button overrides security check
	if(flir_get_safe_boot())
		return 0;

	print_display(CSI "K"); // clear the line
	print_display("\r"); // move cursor back
	print_display(":"); //indicate ready to enter secret code





	for(i=0; i< sizeof(secret)-1; i++)
	{
		if(poll_key(secret[i]))
		{
			print_display("boot"); //failed to enter key
			return 1;
		}
		print_display("."); // entered one key successfully
	}
	print_display("recovery");
	return 0;
}


U_BOOT_CMD(
	kbd_secret, 1, 1, do_kbd_secret,
	"",
	"Returns 0 (true) to shell if success."
);
