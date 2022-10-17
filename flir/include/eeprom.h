/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __FLIR_EEPROM_H
#define __FLIR_EEPROM_H

struct eeprom {
	u8 i2c_bus;
	u16 i2c_address;
	u8 i2c_offset;
	u32 article;
	u32 revision;
	char name[32];
};

int eeprom_read_rev(struct eeprom *eeprom);

#endif
