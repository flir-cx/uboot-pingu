// SPDX-License-Identifier: GPL-2.0+
#include <i2c.h>
#include <env.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include "eeprom.h"

#if !CONFIG_IS_ENABLED(DM_I2C)
#error "Must configure DM_I2C to access EEPROM"
#endif
#if !defined(CONFIG_SYS_I2C_EEPROM_BUS)	|| \
	!defined(CONFIG_SYS_I2C_EEPROM_ADDR)
#error "Must configure CONFIG_SYS_I2C_EEPROM_BUS and CONFIG_SYS_I2C_EEPROM_ADDR"
#endif

/*
 * EEPROM data structures,
 * as specified in doc d1002343
 */

// Product Version, offset 0x00
// 0x40 (64) bytes
struct __packed product_ver {
	char name[20];
	char article[16];
	char serial[10];
	char date[12];
	char revision[4];
	char chksum[2];
};

// CPU board Article Version, offset 0x40
// 0x20 (32) bytes
struct __packed article_ver {
	char article[10];
	char serial[10];
	char revision[4];
	char reserved[6];
	char chksum[2];
};

// MAC Address, offset 0x80
// 0x20 (32) bytes
struct __packed mac_data {
	uchar devid[3];
	char reserved[27];
	u16 chksum;
};

// The complete AT24C02 layout, 256 bytes
struct __packed main_eeprom {
	struct product_ver product;
	struct article_ver article;
	u8 unused_article[32];
	struct mac_data mac;
	u16 core_ver;
	u8 unused_core[30];
	u8 params[16];
	u8 unused_params[47];
	u8 memtest_result;
};

static unsigned int bus = CONFIG_SYS_I2C_EEPROM_BUS;
static unsigned int addr = CONFIG_SYS_I2C_EEPROM_ADDR;
#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_LEN
static const int OFFS_LEN = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;
#else
static const int OFFS_LEN = 1;
#endif

static int eeprom_read_data(unsigned int offset, u8 *data, unsigned int length)
{
	int ret = 0;
	struct udevice *dev;

	ret = i2c_get_chip_for_busnum(bus, (addr >> 1), OFFS_LEN, &dev);
	if (ret != 0) {
		printf("EEPROM chip not found\n\n");
		return ret;
	}

	ret = dm_i2c_read(dev, offset, data, length);
	if (ret != 0)
		printf("Failed to read EEPROM\n");

	return ret;
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
	unsigned int offs = offsetof(struct main_eeprom, article);
	int ret;

	eeprom_select(eeprom->i2c_bus, eeprom->i2c_address);
	ret = eeprom_read_data(offs, (u8 *)&data, sizeof(data));

	if (ret != 0) {
		printf("Read article info from EEPROM failed\n");
		return ret;
	}
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
	unsigned int offs = offsetof(struct main_eeprom, product);
	int ret;

	eeprom_select(eeprom->i2c_bus, eeprom->i2c_address);
	ret = eeprom_read_data(offs, (u8 *)&data, sizeof(data));

	if (ret != 0) {
		printf("Read product info from EEPROM failed\n");
		return ret;
	}
	eeprom->product_number = simple_strtoul(data.article, NULL, 10);
	eeprom->product_revision = simple_strtoul(data.revision, NULL, 10);
	eeprom->product_serial = simple_strtoul(data.serial, NULL, 10);
	memcpy(eeprom->product_name, data.name, sizeof(eeprom->product_name));

	return ret;
}

/**
 * mac_read_from_eeprom - read MAC addresses from EEPROM and write to env
 *
 * This function reads the MAC addresses from EEPROM and sets the
 * appropriate environment variables for each one read.
 *
 * The ethaddr env is a WRITE_ONCE var, that will keep its value forever once
 * it has been set. Subsequent attempts at changing it will fail.
 *
 * Note: Must be called after relocation and is invoked by init_sequence_r
 */
int mac_read_from_eeprom(void)
{
	char ethaddr[18];
	unsigned int offs = offsetof(struct main_eeprom, mac);
	struct mac_data data;

	if (env_get("ethaddr"))
		return 0;

	if (eeprom_read_data(offs, (u8 *)&data, sizeof(data))) {
		// Do not return error, let init_sequence_r proceed
		log_err("%s: Failed to read MAC from EEPROM\n", __func__);
		return 0;
	}

	if (memcmp(data.devid, "\0\0\0", 3) &&
	    memcmp(data.devid, "\xFF\xFF\xFF", 3)) {
		sprintf(ethaddr, "00:40:7F:%02X:%02X:%02X",
			data.devid[0], data.devid[1], data.devid[2]);
		env_set("ethaddr", ethaddr);
		log_info("Saved new ethaddr: %s\n", ethaddr);
	}
	return 0;
}

void eeprom_select(unsigned int i2c_bus, unsigned int i2c_addr)
{
	bus = i2c_bus;
	addr = i2c_addr;
}
