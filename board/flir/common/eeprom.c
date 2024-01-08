// SPDX-License-Identifier: GPL-2.0+
#include <i2c.h>
#include <env.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <linux/delay.h>
#include <u-boot/crc.h>
#include <errno.h>
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

// The complete mainboard AT24C02 layout, 256 bytes
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

/* The complete external (e.g. io board) layout, 256 bytes
 * Only CPU-board EEPROMs store article data on a
 * non-zero offset.
 * struct __packed ext_eeprom {
 *     struct article_ver article;
 *     char data[224];
 * };
 */

#ifdef CONFIG_SYS_I2C_EEPROM_ADDR_LEN
static const int OFFS_LEN = CONFIG_SYS_I2C_EEPROM_ADDR_LEN;
#else
static const int OFFS_LEN = 1;
#endif

#define BUF_SZ sizeof(struct product_ver)
#define PROD_INFO CONFIG_SYS_I2C_EEPROM_BUS,		\
		CONFIG_SYS_I2C_EEPROM_ADDR,		\
		false

struct sup_info {
	char name[10];
	u8 bus;
	u16 address;
	bool ext;
};

/*
 * Registry of supported boards.
 * Any hard-coded info is to be kept in this module.
 * One sunny day in the future, this should all be stored in kconfig or dts.
 */
static struct sup_info boards[] = {
	{"main", PROD_INFO}, {"ec101", PROD_INFO},
	{"ec201", PROD_INFO}, {"ec302", PROD_INFO},
	{"ec401w", PROD_INFO}, {"ec501", PROD_INFO},
	{"eoco", PROD_INFO},
	{"evio", .bus = 2, .address = 0xaa, true}
};

static void hexdump_buffer(u8 *buf, int len)
{
#ifdef DEBUG
	int i;

	printf("EEPROM dump: (%d (0x%02x) bytes)\n", len, len);
	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%02X: ", i);
		printf("%02X ", buf[i]);
		if (((i % 16) == 15) || (i == len - 1))
			printf("\n");
	}
#endif
}

static int eeprom_read_data(unsigned int bus, unsigned int addr,
			    unsigned int offset, u8 *data, unsigned int length)
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

	hexdump_buffer(data, length);
	return ret;
}

static int eeprom_write_data(unsigned int bus, unsigned int addr,
			     unsigned int offset, u8 *data, unsigned int length)
{
	int ret = 0;
	struct udevice *dev;
	void *pos;
	int b_written;

	hexdump_buffer(data, length);
	ret = i2c_get_chip_for_busnum(bus, (addr >> 1), OFFS_LEN, &dev);
	if (ret != 0) {
		printf("EEPROM chip not found\n\n");
		return ret;
	}

	/*
	 * The AT24C02 datasheet says that data can only be written in page
	 * mode, which means 8 bytes at a time, and it takes up to 5ms to
	 * complete a given write.
	 */
	for (b_written = 0, pos = data; b_written < length; b_written += 8, pos += 8) {
		ret = dm_i2c_write(dev, offset + b_written, pos, min((int)length - b_written, 8));
		if (ret)
			break;
		mdelay(5);
	}

	return ret;
}

/*
 * eeprom_supported_board() - Is this board known to us?
 * @name: article string name
 * Return: -ENODEV if name not found, >=0 on success
 */
static int eeprom_supported_board(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(boards); i++)
		if (!strcmp(boards[i].name, name))
			return i;
	return -ENODEV;
}

static int eeprom_do_read(const char *name, struct hw_version *info,
			  u8 *buf, bool prodinfo)
{
	int ret;
	int ix = eeprom_supported_board(name);
	unsigned int offs = 0;

	if (!info)
		return -EINVAL;

	if (ix < 0) {
		log_err("Board '%s' is not supported\n", name);
		return -EINVAL;
	}

	if (prodinfo) {
		if (boards[ix].ext) {
			log_err("No Product Info on non-CPU boards\n");
			return -EINVAL;
		}
	} else {
		if (!boards[ix].ext)
			offs = offsetof(struct main_eeprom, article);
	}
	ret = eeprom_read_data(boards[ix].bus, boards[ix].address, offs,
			       buf, BUF_SZ);
	return ret;
}

int eeprom_read_rev(const char *name, struct hw_version *info)
{
	struct article_ver *art;
	u8 buf[BUF_SZ] = {0};
	int ret = eeprom_do_read(name, info, buf, false);

	if (ret)
		return ret;
	art = (struct article_ver *)buf;
	info->article = simple_strtoul(&art->article[1], NULL, 10);
	info->revision = simple_strtoul(art->revision, NULL, 10);
	info->serial = simple_strtoul(art->serial, NULL, 10);
	return 0;
}

int eeprom_read_product(struct hw_version *info)
{
	struct product_ver *prod;
	u8 buf[BUF_SZ] = {0};
	int ret = eeprom_do_read("main", info, buf, true);

	if (ret)
		return ret;
	prod = (struct product_ver *)buf;
	memcpy(info->name, prod->name, sizeof(info->name));
	info->article = simple_strtoul(&prod->article[1], NULL, 10);
	info->revision = simple_strtoul(prod->revision, NULL, 10);
	info->serial = simple_strtoul(prod->serial, NULL, 10);
	return 0;
}

int eeprom_read_mac(struct mac *addr)
{
	const u8 oui[3] = {0, 0x40, 0x7f};
	struct mac_data data;
	u16 stored_crc = 0;
	unsigned int offs = offsetof(struct main_eeprom, mac);
	int ret = eeprom_read_data(CONFIG_SYS_I2C_EEPROM_BUS,
				   CONFIG_SYS_I2C_EEPROM_ADDR,
				   offs,
				   (u8 *)&data, sizeof(data));

	if (ret) {
		log_err("%s: Failed to read MAC from EEPROM\n", __func__);
		return 1;
	}

	stored_crc = data.chksum;
	data.chksum = 0;
	data.chksum = crc16_ccitt(0, (u8 *)&data, 32);
	if (stored_crc != data.chksum)
		log_err("%s: MAC DeviceID (OUI) CRC is incorrect\n", __func__);

	memcpy(&addr->b[0], oui, 3);
	memcpy(&addr->b[3], data.devid, 3);

	return 0;
}

int eeprom_write_mac(struct mac *addr)
{
	struct mac_data data, verify_data;
	unsigned int offs = offsetof(struct main_eeprom, mac);
	int ret;

	memset(&data, 0, sizeof(data));
	memcpy(&data, &addr->b[3], 3);
	data.chksum = crc16_ccitt(0, (u8 *)&data, 32);

	ret = eeprom_write_data(CONFIG_SYS_I2C_EEPROM_BUS,
				CONFIG_SYS_I2C_EEPROM_ADDR,
				offs,
				(u8 *)&data, sizeof(data));
	if (ret)
		return 1;

	ret = eeprom_read_data(CONFIG_SYS_I2C_EEPROM_BUS,
			       CONFIG_SYS_I2C_EEPROM_ADDR,
			       offs,
			       (u8 *)&verify_data, sizeof(verify_data));
	if (ret || memcmp(&verify_data, &data, sizeof(data)))
		log_err("%s: EEPROM write verification failed\n", __func__);

	return 0;
}

int eeprom_read_rev_generic(unsigned int bus, unsigned int address, unsigned int offset,
			    struct hw_version *info)
{
	struct article_ver *art;
	int ret = 0;
	u8 buf[BUF_SZ] = {0};

	ret = eeprom_read_data(bus, address, offset, buf, sizeof(buf));
	if (ret)
		return ret;

	art = (struct article_ver *)buf;
	info->article = simple_strtoul(&art->article[1], NULL, 10);
	info->revision = simple_strtoul(art->revision, NULL, 10);
	info->serial = simple_strtoul(art->serial, NULL, 10);

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
	struct mac_data data;
	unsigned int bus = CONFIG_SYS_I2C_EEPROM_BUS;
	unsigned int addr = CONFIG_SYS_I2C_EEPROM_ADDR;
	unsigned int offs = offsetof(struct main_eeprom, mac);

	if (env_get("ethaddr"))
		return 0;

	if (eeprom_read_data(bus, addr, offs, (u8 *)&data, sizeof(data))) {
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
