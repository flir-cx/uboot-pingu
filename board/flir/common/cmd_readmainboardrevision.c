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
#include <vsprintf.h>
#include <command.h>
#include "eeprom.h"
#include "cmd_kbd.h"

/*
 * Note: Board "main" means whatever board that has been setup using
 * CONFIG_SYS_I2C_EEPROM_BUS and CONFIG_SYS_I2C_EEPROM_ADDR
 */
#define MAIN_BOARD "main"

static int get_mainboard_version(int *article, int *revision)
{
	static struct board_info cache;
	int ret = 0;

	if (!cache.article)
		ret = eeprom_read_rev(MAIN_BOARD, &cache);

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
	   "Read the revision of the CPU (main) board",
	   "Read the revision of the CPU (main) board\n"
	);

U_BOOT_CMD(readmainboardarticle, CONFIG_SYS_MAXARGS, 0, do_readmainboardarticle,
	   "Read the article of the CPU (main)  board",
	   "Read the article of the CPU (main) board\n"
	);

