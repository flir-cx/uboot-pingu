#include <../../../flir/include/cmd_kbd.h>
#include <common.h>
#include <command.h>
#include <env.h>
#include <asm/gpio.h>
#include <i2c.h>

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (buttons[i].get_key && !buttons[i].get_key(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

#define PCA9534_INPUT_PORT			0x0

int get_gpio_ioexpander(unsigned combnr)
/* combnr is (addr << 8 + nr)
   needs to be one parameter to be compatible with gpio_get_value() */
{
	u8 buf;
        u8 addr = combnr >> 8;
        unsigned nr = combnr & 0xff;
        
	i2c_set_bus_num(2);
	if(i2c_read( addr, PCA9534_INPUT_PORT, 1, &buf, 1))
		return -1;

	return (buf & (1 << nr) );
}

static int do_kbd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	env_set("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

