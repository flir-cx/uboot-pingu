// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 FLIR Systems.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <command.h>
#include "../../../flir/include/eeprom.h"
#include "../../../flir/include/cmd_kbd.h"

#define EEPROM_BUS_ID (2)
#define MAIN_EEPROM_I2C_ADDR (0xae)
#define MAIN_EEPROM_I2C_OFFS (0x40)

static int get_mainboard_version(int *article, int *revision)
{
	static struct eeprom cache = {
		.i2c_bus = EEPROM_BUS_ID,
		.i2c_address = MAIN_EEPROM_I2C_ADDR,
		.i2c_offset = MAIN_EEPROM_I2C_OFFS,
		.article = 0,
		.revision = 0
	};
	int ret = 0;

	if (!cache.article)
		ret = eeprom_read_rev(&cache);

	*article = cache.article;
	*revision = cache.revision;

	return ret;
}

static int do_readmainboardarticle(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int article;
	int revision;
	char str[20];

	get_mainboard_version(&article, &revision);
	printf("Mainboard article %i\n ", article);
	snprintf(str, strlen(str), "%i", article);
	env_set("mainboardarticle", str);

	return 0;
}

static int do_readmainboardrevision(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int article;
	int revision;
	char str[20];

	get_mainboard_version(&article, &revision);
	printf("Mainboard revision %i\n ", revision);
	snprintf(str, strlen(str), "%i", revision);
	env_set("mainboardrevision", str);

	return 0;
}

U_BOOT_CMD(readmainboardrevision, CONFIG_SYS_MAXARGS, 0, do_readmainboardrevision,
	   "Read the revision of the ec101 mainboard",
	   "Read the revision of the ec101 mainboard\n"
	);

U_BOOT_CMD(readmainboardarticle, CONFIG_SYS_MAXARGS, 0, do_readmainboardarticle,
	   "Read the article of the ec101 mainboard",
	   "Read the article of the ec101 mainboard\n"
	);

