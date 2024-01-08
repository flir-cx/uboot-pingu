// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <common.h>
#include <command.h>
#include <ctype.h>
#include "eeprom.h"

static struct mac addr = {{ 0x00, 0x40, 0x7f, 0, 0, 0 }};

static void mac_print(void)
{
	printf("Eth: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       addr.b[0], addr.b[1], addr.b[2], addr.b[3], addr.b[4], addr.b[5]);
}

static int mac_read(void)
{
	if (eeprom_read_mac(&addr))
		return 1;

	return 0;
}

static int mac_write(void)
{
	if (eeprom_write_mac(&addr))
		return 1;

	return 0;
}

static int nic_set(int argc, char *arg)
{
	char *p = arg;
	u8 nic[3];
	int bpos = 0;

	while ((*p != '\0') && (bpos < 3)) {
		if (!isxdigit(*p)) {
			log_err("Error: Malformed MAC, illegal character\n");
			return 1;
		}
		nic[bpos] = (u8)simple_strtoul(p, &p, 16);
		bpos++;
		if (*p == ':')
			p++;
	};
	if (bpos < 3) {
		log_err("Error: Malformed MAC, too few bytes\n");
		return 1;
	}
	memcpy(&addr.b[3], nic, 3);

	return 0;
}

int do_mac(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char cmd = 'p';
	int ret = 0;

	if (argc > 1)
		cmd = argv[1][0];

	switch (cmd) {
	case 'p':
		mac_print();
		break;
	case 'r':
		ret = mac_read();
		break;
	case 'w':
		ret = mac_write();
		break;
	case '0' ... '9':
	case 'a' ... 'f':
	case 'A' ... 'F':
		ret = nic_set(argc, &argv[1][0]);
		break;
	default:
		log_err("Illegal mac argument '%c'\n", cmd);
		ret = 1;
	}

	return ret;
}

U_BOOT_CMD(mac, 3, 1, do_mac,
	   "Display and program the MAC address in EEPROM",
	   "[r|w|XX:XX:XX]\n"
	   "FLIR vendor (OUI) bytes are always 00:40:7f, only NIC bytes need to be specified.\n"
	   "mac          - print cached MAC\n"
	   "mac r        - read MAC from EEPROM to cache\n"
	   "mac w        - write cached MAC to EEPROM\n"
	   "mac XX:XX:XX - store new MAC in cache (Device/NIC bytes only)\n"
	);
