// SPDX-License-Identifier: GPL-2.0+
#include <i2c.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include "../../../flir/include/cmd_kbd.h"
#include "../../../flir/include/eeprom.h"

#if ! CONFIG_IS_ENABLED(DM_I2C)
#error "Must configure DM_I2C to access EEPROM"
#endif

/*
 * Raw hardware version struct (in EEPROM)
 */
struct hardware_version {
	char article[10];
	char serial[10];
	char revision[4];
	char module_offset[2]; // Address of a parent module, ModRevstruct
	char module_device[2]; // i2c device
	char reserved[2];
	// 2B checksum
};

/*
 * Read EEPROM. The supplied argument must define
 * bus, address (a.k.a chip) and offset (a.k.a address)
 *
 * Article and revision are parsed and filled in.
 *
 * Return 0 on success, <0 on fail
 */
int eeprom_read_rev(struct eeprom *eeprom)
{
	int ret = 0;
	int bus_no = eeprom->i2c_bus;
	int chip = eeprom->i2c_address >> 1;
	int addr = eeprom->i2c_offset;
	const int addr_len = 1;
	const int offs_len = 1;
	struct hardware_version hwrev;
	struct udevice *bus;
	struct udevice *dev;

	ret = uclass_get_device_by_seq(UCLASS_I2C, bus_no, &bus);
	return_on_status(ret, "No i2c bus 2 found\n");

	ret = i2c_get_chip(bus, chip, addr_len, &dev);
	return_on_status(ret, "Chip 0x%02x not found\n", chip);

	ret = i2c_set_chip_offset_len(dev, offs_len);
	return_on_status(ret, "Set offset_len on 0x%02x failed\n", chip);

	ret = dm_i2c_read(dev, addr, (u8 *)&hwrev, sizeof(hwrev));
	return_on_status(ret, "Read EEPROM failed\n");

	eeprom->article = simple_strtoul(&hwrev.article[1], NULL, 10);
	eeprom->revision = simple_strtoul(hwrev.revision, NULL, 10);

	return ret;
}
