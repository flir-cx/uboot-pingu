// SPDX-License-Identifier: GPL-2.0+
#include <common.h>
#include <command.h>
#include <env.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include "../../../flir/include/cmd_kbd.h"

#if ! CONFIG_IS_ENABLED(DM_I2C)
#error "Must configure DM_I2C to use GPIO expander"
#endif

static int get_gpio_ioexpander(unsigned int nr);

struct button_key {
	char const      *name;
	int             (*get_key)(unsigned int nr);
	unsigned int    gpnum;
	char            ident;
	struct udevice  *dev;
};

#define KEYBOARD_IO_EXP_I2C_ADDR        0x20
#define KEYBOARD_BEIA_IO_EXP_I2C_ADDR   0x21
#define KEYBOARD_IO_EXP_BUS_NUM         2
#define PCA9534_INPUT_PORT              0x0
#define PCA9534_ADDR_LEN                1
#define PCA9534_OFFSET_LEN              1

#define EVIOBUSMSK (KEYBOARD_IO_EXP_I2C_ADDR << 8)
#define BEIABUSMSK (KEYBOARD_BEIA_IO_EXP_I2C_ADDR << 8)

static struct button_key buttons[] = {
  //	{"sw_load",	gpio_get_value, IMX_GPIO_NR(7, 11),	'S', NULL },
	{"right",	get_gpio_ioexpander, EVIOBUSMSK | 0,	'R', NULL },
	{"left",	get_gpio_ioexpander, EVIOBUSMSK | 1,	'L', NULL },
	{"up",		get_gpio_ioexpander, EVIOBUSMSK | 2,	'U', NULL },
	{"back",	get_gpio_ioexpander, EVIOBUSMSK | 4,	'B', NULL },
	{"down",	get_gpio_ioexpander, EVIOBUSMSK | 5,	'D', NULL },
	{"middle",	get_gpio_ioexpander, EVIOBUSMSK | 6,	'M', NULL },
	{"sw_load",	get_gpio_ioexpander, BEIABUSMSK | 0,    'S', NULL },
};

static int kbd_is_initialized;
static char kbuf[ARRAY_SIZE(buttons) + 1];

/*
 * Find and initialize all possible i2c gpio expanders.
 * Update the buttons array with valid devices.
 */
static int init_kbd_devices(void)
{
	int ret;
	int i;
	struct udevice *bus;
	struct udevice *dev;

	if (kbd_is_initialized)
		return 0;

	ret = uclass_get_device_by_seq(UCLASS_I2C, KEYBOARD_IO_EXP_BUS_NUM, &bus);
	return_on_status(ret, "No i2c bus 2 found\n");

	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		int addr = (buttons[i].gpnum >> 8);

		if (buttons[i].get_key != get_gpio_ioexpander)
			continue;

		ret = i2c_get_chip(bus, addr, PCA9534_ADDR_LEN, &dev);
		return_on_status(ret, "chip 0x%02x not found\n", addr);
		ret = i2c_set_chip_offset_len(dev, PCA9534_OFFSET_LEN);
		return_on_status(ret, "set offset_len on 0x%02x failed\n", addr);
		buttons[i].dev = dev;
	}
	kbd_is_initialized = 1;

	return 0;
}

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed, or a negative error
 *
 * Return: Number of keys pressed, or <0 on error
 *         Registered keys are stored in the kbuf
 */
int read_keys(char **buf)
{
	int numpressed = 0;
	int i;
	int ret = init_kbd_devices();

	memset(kbuf, 0, sizeof(kbuf));
	*buf = kbuf;
	return_on_status(ret, "Failed to init kbd dev\n");
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!buttons[i].get_key)
			continue;
		ret = buttons[i].get_key(buttons[i].gpnum);
		if (ret == 0)
			kbuf[numpressed++] = buttons[i].ident;
		cond_log_return(ret < 0, ret,
				"get_key failed for key '%c'\n", buttons[i].ident);
	}

	return numpressed;
}

/*
 * Check if button with given combnr is pressed.
 *
 * combnr is (addr << 8 + nr)
 * Function prototype needs to be compatible with gpio_get_value()
 *
 * Return: 0 when the requested button was pressed
 *         >0 when no button is pressed
 *         <0 when HW read failed
 */
static int get_gpio_ioexpander(unsigned int combnr)
{
	const int data_len = 1;
	unsigned int nr = combnr & 0xff;
	u8 buf[2] = {0xff, 0xff};
	int i;
	int ret = 0;

	for (i = 0; i < ARRAY_SIZE(buttons); i++)
		if (buttons[i].gpnum == combnr && buttons[i].dev)
			ret = dm_i2c_read(buttons[i].dev, PCA9534_INPUT_PORT, buf, data_len);

	return_on_status(ret, "read failed, status %d\n", ret);

	return buf[0] & (1 << nr);
}

static int do_kbd(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char *buf;
	int numpressed = read_keys(&buf);

	env_set("keybd", buf);
	return numpressed == 0;
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);
#endif

U_BOOT_CMD(
	flir_kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);
