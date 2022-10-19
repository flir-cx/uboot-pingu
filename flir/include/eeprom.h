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
};

int eeprom_read_rev(struct eeprom *eeprom);
int eeprom_read_product(struct eeprom *eeprom);

#endif
