// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <command.h>
#include <stdio_dev.h>
#include <dm.h>
#include <i2c.h>
#include <asm/arch/pcc.h>

static int get_eeprom_device(struct udevice **dev)
{
	static struct udevice *eeprom_dev;
	int ret = 0;

	if (!eeprom_dev) {
		ret = i2c_get_chip_for_busnum(6, 0x57, 1, &eeprom_dev);
		if (ret)
			printf("Cannot find eeprom: %d\n", ret);
	}

	*dev = eeprom_dev;

	return ret;
}

int get_board_serial(uint8_t *buf)
{
	struct udevice *dev;
	int ret;

	ret = get_eeprom_device(&dev);
	if (ret) {
		printf("Can't set mac address\n");
		return ret;
	}

	ret = dm_i2c_read(dev, 0x4A, buf, 8);
	buf[8] = '\0';

	if (ret)
		printf("Couldn't read from eeprom: %d\n", ret);

	return ret;
}

int setmac_func(bool bt)
{
	char addr[18];
	u32 addrI = 0;
	char buf[16];

	int res = get_board_serial((uint8_t *)buf);

	if (!res) {
		strcpy(addr, "00:40:7f:00:00:00");

		// convert to integer
		for (int i = 0; buf[i] != '\0'; ++i)
			addrI = addrI * 10 + buf[i] - '0';

		addrI = addrI << 1; // shift up one to allow for different bt and usb mac address
		if (bt)
			addrI++;

		snprintf(buf, 10, "%x", addrI);
		int len = strlen(buf);

		//address byte 6
		addr[16] = buf[len - 1];
		addr[15] = buf[len - 2];
		//address byte 5
		addr[13] = buf[len - 3];
		addr[12] = buf[len - 4];
		//address byte 4
		addr[10] = buf[len - 5];
	}

	if (res)
		strcpy(addr, bt ? "00:04:f3:ff:ff:fc" : "00:04:f3:ff:ff:fb");

	env_set(bt ? "btaddr" : "wlanaddr", addr);

	return 0;
}

int do_setmac(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	setmac_func(false);
	setmac_func(true);

	return 0;
}

U_BOOT_CMD(setmac, 2, 1, do_setmac,
	   "setmac",
	   "setmac"
);
