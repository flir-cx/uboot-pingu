// SPDX-License-Identifier: GPL-2.0+
#include <command.h>
#include <errno.h>
#include <vsprintf.h>
#include <stdio_dev.h>
#include <video.h>
#include <video_font.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <i2c.h>

#define ESC "\x1b"
#define CSI "\x1b["
#define CLR_LINE CSI "2K"

// TODO: Centralize EEPROM address info
#define EEPROM_BUS_ID (0)
#define MAIN_EEPROM_I2C_ADDR (0xae)
#define MAIN_EEPROM_I2C_OFFS (0x40)

static struct stdio_dev *sdev;
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

// Product Version
// 0x40 (64) bytes
struct product_ver {
	char name[20];
	char article[16];
	char serial[10];
	char date[12];
	char revision[4];
	char chksum[2];
};

#define PRODUCT_DATA_OFFSET (0x00)

// CPU board Article Version
// 0x20 (32) bytes
struct article_ver {
	char article[10];
	char serial[10];
	char revision[4];
	char reserved[6];
	char chksum[2];
};

static int do_printboardinfo(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct udevice *dev;
	struct product_ver proddata;
	int ret;

	if (init_stdio())
		printf("Stdio error, proceed without visual feedback\n");

	ret = i2c_get_chip_for_busnum(EEPROM_BUS_ID, (MAIN_EEPROM_I2C_ADDR >> 1), 1, &dev);
	if (ret)
		goto err_no_i2c;
	ret = dm_i2c_read(dev, PRODUCT_DATA_OFFSET, (uint8_t *)&proddata, sizeof(proddata));
	if (ret)
		goto err_no_i2c;

	print_display(" ");
	print_display(proddata.name);
	print_display(" ");
	print_display(proddata.article);
	print_display("-");
	print_display(proddata.revision);
	print_display(" ");
	print_display(proddata.serial);
	print_display(" ");
	print_display("\n");
	printf("%s %s-%s %s\n", proddata.name, proddata.article, proddata.revision,proddata.serial);
	return ret;

err_no_i2c:
	printf("I2C Read error\n");
	print_display("I2C Read error");
	return ret;
}

U_BOOT_CMD(printboardinfo, 1, 1, do_printboardinfo,
	   "print main board product information on console (and on LCD if detected)",
	   "Returns 0 (true) to shell if success."
);
