/*
 * Copyright 2006, 2008-2009, 2011 Freescale Semiconductor
 * York Sun (yorksun@freescale.com)
 * Haiying Wang (haiying.wang@freescale.com)
 * Timur Tabi (timur@freescale.com)
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <command.h>
#include <i2c.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <u-boot/crc.h>

/**
 * static eeprom: EEPROM layout for FLIR mainboard
 *
 */
static struct __attribute__ ((__packed__)) eeprom {
	u8 prodVer[64];
	u8 cpuVer[64];
	u8 mac[3];
	u8 macEmpty[27];
	u16 macCrc;
	u8 zcoreVer[16];
	u8 dummy[80];
} e;

#define MAX_NUM_PORTS 1

/* Set to 1 if we've read EEPROM into memory */
static int has_been_read = 0;

/**
 * show_eeprom - display the contents of the EEPROM
 */
static void show_eeprom(void)
{
	/* Show MAC addresses  */
	u8 *p = &e.mac[0];
	printf("Eth: %02x:%02x:%02x:%02x:%02x:%02x\n",
		0x00, 0x40, 0x7f, p[0],	p[1], p[2]);

#ifdef DEBUG
	int i;
	printf("EEPROM dump: (0x%x bytes)\n", sizeof(e));
	for (i = 0; i < sizeof(e); i++) {
		if ((i % 16) == 0)
			printf("%02X: ", i);
		printf("%02X ", ((u8 *)&e)[i]);
		if (((i % 16) == 15) || (i == sizeof(e) - 1))
			printf("\n");
	}
#endif
}

/**
 * read_eeprom - read the EEPROM into memory
 */
static int read_eeprom(void)
{
	int ret;
	u16 tempcrc;
#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	unsigned int bus;
#endif

#if 0
	if (has_been_read)
		return 0;
#endif
#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_SYS_EEPROM_BUS_NUM);
#endif

	ret = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, CONFIG_SYS_I2C_EEPROM_ADDR_LEN,
		(void *)&e, sizeof(e));


#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	i2c_set_bus_num(bus);
#endif

#ifdef DEBUG
	show_eeprom();
#endif
	tempcrc = e.macCrc;
	e.macCrc = 0;
	e.macCrc = crc16_ccitt(0, &e.mac[0], 32);
	if (tempcrc != e.macCrc) {
		printf("EEPROM: Mac Address CRC incorrect\n");
		memset(&e.mac[0], 0, 32);
	}

	has_been_read = (ret == 0) ? 1 : 0;

	return ret;
}

/**
 * prog_eeprom - write the EEPROM from memory
 */
static int prog_eeprom(void)
{
	int ret = 0;
	int i;
	void *p;
#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	unsigned int bus;
#endif

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_SYS_EEPROM_BUS_NUM);
#endif
	memset(&e.macEmpty[0], 0, 27);
	e.macCrc = 0;
	e.macCrc = crc16_ccitt(0, &e.mac[0], 32);

	/*
	 * The AT24C02 datasheet says that data can only be written in page
	 * mode, which means 8 bytes at a time, and it takes up to 5ms to
	 * complete a given write.
	 */
	for (i = 0, p = &e; i < sizeof(e); i += 8, p += 8) {
		ret = i2c_write(CONFIG_SYS_I2C_EEPROM_ADDR, i, CONFIG_SYS_I2C_EEPROM_ADDR_LEN,
			p, min((int)(sizeof(e) - i), 8));
		if (ret)
			break;
		udelay(5000);	/* 5ms write cycle timing */
	}

	if (!ret) {
		/* Verify the write by reading back the EEPROM and comparing */
		struct eeprom e2;

		ret = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0,
			CONFIG_SYS_I2C_EEPROM_ADDR_LEN, (void *)&e2, sizeof(e2));
		if (!ret && memcmp(&e, &e2, sizeof(e)))
			ret = -1;
	}

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	i2c_set_bus_num(bus);
#endif

	if (ret) {
		printf("Programming failed.\n");
		has_been_read = 0;
		return -1;
	}

	printf("Programming passed.\n");
	return 0;
}

/**
 * h2i - converts hex character into a number
 *
 * This function takes a hexadecimal character (e.g. '7' or 'C') and returns
 * the integer equivalent.
 */
static inline u8 h2i(char p)
{
	if ((p >= '0') && (p <= '9'))
		return p - '0';

	if ((p >= 'A') && (p <= 'F'))
		return (p - 'A') + 10;

	if ((p >= 'a') && (p <= 'f'))
		return (p - 'a') + 10;

	return 0;
}

/**
 * set_mac_address - stores a MAC address into the EEPROM
 *
 * This function takes a pointer to MAC address string
 * (i.e."XX:XX:XX:XX:XX:XX", where "XX" is a two-digit hex number) and
 * stores it in one of the MAC address fields of the EEPROM local copy.
 */
static void set_mac_address(unsigned int index, const char *string)
{
	char *p = (char *) string;
	unsigned int i;

	if ((index >= MAX_NUM_PORTS) || !string) {
		printf("Usage: mac <n> XX:XX:XX:XX:XX:XX\n");
		return;
	}

	for (i = 0; *p && (i < 6); i++) {
		if (i > 2)
		    e.mac[i-3] = simple_strtoul(p, &p, 16);
		else {
		    (void)simple_strtoul(p, &p, 16);
		}
		if (*p == ':')
			p++;
	}
}

int do_mac(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char cmd;

	if (argc == 1) {
		show_eeprom();
		return 0;
	}

	cmd = argv[1][0];

	if (cmd == 'r') {
		read_eeprom();
		return 0;
	}

	if (argc == 2) {
		switch (cmd) {
		case 's':	/* save */
			prog_eeprom();
			break;
		default:
			return cmd_usage(cmdtp);
		}

		return 0;
	}

	/* We know we have at least one parameter  */

	switch (cmd) {
	case '0' ... '9':	/* "mac 0" through "mac 22" */
		set_mac_address(simple_strtoul(argv[1], NULL, 10), argv[2]);
		break;
	case 'h':	/* help */
	default:
		return cmd_usage(cmdtp);
	}

	return 0;
}

/**
 * mac_read_from_eeprom - read the MAC addresses from EEPROM
 *
 * This function reads the MAC addresses from EEPROM and sets the
 * appropriate environment variables for each one read.
 *
 * The environment variables are only set if they haven't been set already.
 * This ensures that any user-saved variables are never overwritten.
 *
 * This function must be called after relocation.
 */
int mac_read_from_eeprom(void)
{
	if (read_eeprom()) {
		printf("Read failed.\n");
		return -1;
	}

	if (memcmp(&e.mac[0], "\0\0\0", 6) &&
	    memcmp(&e.mac[0], "\xFF\xFF\xFF", 6)) {
		char ethaddr[18];

		sprintf(ethaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
			0x00,
			0x40,
			0x7f,
			e.mac[0],
			e.mac[1],
			e.mac[2]);
		/* Only initialize environment variables that are blank
		 * (i.e. have not yet been set)
		 */
		if (!env_get("ethaddr")) {
			env_set("ethaddr", ethaddr);
		}
	}

	return 0;
}

U_BOOT_CMD(
		mac, 3, 1,  do_mac,
		"display and program the system ID and MAC addresses in EEPROM",
		"[read|save|0|1|2|3|4|5|6|7]\n"
		"mac read\n"
		"    - read EEPROM content into memory\n"
		"mac save\n"
		"    - save to the EEPROM\n"
		"mac X\n"
		"    - program the MAC address for port X [X=0...7]"
		);

