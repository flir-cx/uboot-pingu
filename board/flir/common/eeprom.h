/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __FLIR_EEPROM_H
#define __FLIR_EEPROM_H

struct eeprom {
	u8 i2c_bus;
	u16 i2c_address;
	u8 i2c_offset;

	char product_name[20];
	u32 product_number;
	u32 product_serial;
	u32 product_revision;

	u32 article_number;
	u32 article_serial;
	u32 article_revision;

	char name[32];
	u8 mac[6];
	u16 mac_crc;
};

/**
 * eeprom_set_addr() - set non-default eeprom address
 *
 * Mainboard I2C EEPROM is configured using SYS_I2C_EEPROM_BUS
 * and SYS_I2C_EEPROM_ADDR. Use this function to select alternate
 * addresses.
 */
void eeprom_select(unsigned int bus, unsigned int addr);

int eeprom_read_rev(struct eeprom *eeprom);
int eeprom_read_product(struct eeprom *eeprom);

#endif
