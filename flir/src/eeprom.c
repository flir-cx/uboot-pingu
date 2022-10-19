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
 * EEPROM data structures,
 * as specified in doc d1002343
 */

// Product Version
// 0x40 (64) bytes
struct product_ver {
	char name[20];
	char article[16];
	char serial[10];
	char date[12];
	char revision[4];
	char chksum[2];
};

// CPU board Article Version
// 0x20 (32) bytes
struct article_ver {
	char article[10];
	char serial[10];
	char revision[4];
	char reserved[6];
	char chksum[2];
};

#define PRODUCT_DATA_OFFSET (0x00)

static int eeprom_read_data(struct eeprom *eeprom, unsigned int offset,
			    u8 *data, unsigned int length)
{
	int ret = 0;
	const int I2C_OFFS_LEN = 1;
	struct udevice *dev;

	ret = i2c_get_chip_for_busnum(eeprom->i2c_bus, (eeprom->i2c_address >> 1),
				      I2C_OFFS_LEN, &dev);
	return_on_status(ret, "EEPROM chip not found\n\n");

	ret = dm_i2c_read(dev, offset, data, length);
	return_on_status(ret, "Failed to read EEPROM\n");

	return 0;
}

/*
 * Read article data from the EEPROM.
 * The supplied argument must define bus and address (a.k.a chip).
 * (Offset will be ignored)
 * Article data is parsed and written back to the eeprom struct.
 *
 * Return 0 on success, <0 on fail
 */
int eeprom_read_rev(struct eeprom *eeprom)
{
	struct article_ver data;
	int ret = eeprom_read_data(eeprom, eeprom->i2c_offset, (u8 *)&data, sizeof(data));

	return_on_status(ret, "Read article info from EEPROM failed\n");
	eeprom->article_number = simple_strtoul(&data.article[1], NULL, 10);
	eeprom->article_revision = simple_strtoul(data.revision, NULL, 10);
	eeprom->article_serial = simple_strtoul(data.serial, NULL, 10);

	return ret;
}

/*
 * Read product data from the EEPROM.
 * The supplied argument must define bus and address (a.k.a chip).
 * (Offset will be ignored)
 * Product data is parsed and written back to the eeprom struct.
 *
 * Return 0 on success, <0 on fail
 */
int eeprom_read_product(struct eeprom *eeprom)
{
	struct product_ver data;
	int ret = eeprom_read_data(eeprom, PRODUCT_DATA_OFFSET, (u8 *)&data, sizeof(data));

	return_on_status(ret, "Read product info from EEPROM failed\n");
	eeprom->product_number = simple_strtoul(data.article, NULL, 10);
	eeprom->product_revision = simple_strtoul(data.revision, NULL, 10);
	eeprom->product_serial = simple_strtoul(data.serial, NULL, 10);
	memcpy(eeprom->product_name, data.name, sizeof(eeprom->product_name));

	return ret;
}

