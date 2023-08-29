/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __FLIR_EEPROM_H
#define __FLIR_EEPROM_H

struct board_info {
	char name[20];
	u32 article;
	u32 serial;
	u32 revision;
};

/**
 * eeprom_read_rev() - Read article info from EEPROM
 * @name: board name, e.g. ec101, ec302, evio
 * @info: info that is filled in on success
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_rev(const char *name, struct board_info *info);

/**
 * eeprom_read_product() - Read product info from main-board EEPROM
 * @info: info that is filled in on success
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_product(struct board_info *info);

/**
 * eeprom_read_rev_generic() - Read article info from EEPROM
 *
 * Read board info from EEPROM, using its explicit I2C info.
 * Try not to use this. Offset info is maintained in the module.
 *
 * @bus: I2C bus number
 * @address: I2C chip address
 * @offset: data ofset on chip (only 0x0 and 0x40 allowed..)
 * @info: return data
 *
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_rev_generic(unsigned int bus, unsigned int address, unsigned int offset,
			    struct board_info *info);

#endif
