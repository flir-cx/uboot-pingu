/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __FLIR_EEPROM_H
#define __FLIR_EEPROM_H

struct hw_version {
	char name[20];
	u32 article;
	u32 serial;
	u32 revision;
};

struct mac {
	u8 b[6];
};

/**
 * eeprom_read_rev() - Read article info from EEPROM
 * @name: board name, e.g. ec101, ec302, evio
 * @info: info that is filled in on success
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_rev(const char *name, struct hw_version *info);

/**
 * eeprom_read_product() - Read product info from main-board EEPROM
 * @info: info that is filled in on success
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_product(struct hw_version *info);

/**
 * eeprom_read_mac() - Read MAC NIC from main EEPROM
 * Return: 0 on success, <0 on failure
 */
int eeprom_read_mac(struct mac *addr);

/**
 * eeprom_write_mac() - Write MAC to main EEPROM
 * Vendor (OUI) will be set to 00:40:7f (FLIR Systems)
 * Return: 0 on success, <0 on failure
 */
int eeprom_write_mac(struct mac *addr);

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
			    struct hw_version *info);

#endif
