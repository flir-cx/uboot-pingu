// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 FLIR Systems.
 */
#include <log.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <linux/delay.h>
#include <errno.h>
#include "bq27xxx.h"

#define BQ_UNDEFINED (-1)
#define TYPE_BQ27520 (0x520)
#define TYPE_BQ27542 (0x542)

// Common Control Subcommands
#define CMD_CONTROL_STATUS (0x0000)
#define CMD_DEVICE_TYPE    (0x0001)
#define CMD_FW_VERSION     (0x0002)
#define CMD_CHEM_ID        (0x0008)
#define CMD_SET_HIBERNATE  (0x0011)
#define CMD_CLEAR_HIBERNATE (0x0012)
#define CMD_RESET          (0x0041)

// Device-specific Control Subcommands
#define BQ27520_CMD_DF_VERSION (0x1f)
#define BQ27542_CMD_DF_VERSION (0x0c)

// Common command codes (i.e. reg start addresses)
#define REG_CONTROL              (0x00)
#define REG_AT_RATE              (0x02)
#define REG_TEMPERATURE          (0x06)
#define REG_VOLTAGE              (0x08)
#define REG_INTERNAL_TEMPERATURE (0x28)

// Device-specific command codes
#define BQ27520_REG_STATE_OF_CHARGE (0x20)
#define BQ27542_REG_STATE_OF_CHARGE (0x2c)
#define BQ27520_REG_DESIGN_CAPACITY (0x2e)
#define BQ27542_REG_DESIGN_CAPACITY (0x3c)

// Extended Data Commands
#define REG_DATA_FLASH_CLASS  (0x3E)
#define REG_DATA_FLASH_BLOCK  (0x3F)
#define REG_BLOCK_DATA        (0x40)
#define BLOCK_DATA_MAXLEN     (32)
#define MAN_INFO_CLASS        (0x39)

// Control Status Bits
#define BIT_SS         (13)
#define BIT_INITCOMP   (7)

static struct udevice *dev;
static struct {
	int type;
	int bus;
	int chip;
} gauge = {BQ_UNDEFINED, BQ_UNDEFINED, BQ_UNDEFINED};

static int bq_read_regpair(u8 reg, u16 *val);
static int bq_write_regpair(u8 reg, u16 val);

static bool bq_is_initialized(void)
{
	return gauge.type != BQ_UNDEFINED &&
		gauge.bus != BQ_UNDEFINED &&
		gauge.chip != BQ_UNDEFINED;
}

static int bq_init(void)
{
	int ret;
	u16 type;

	gauge.bus = CONFIG_FLIR_BQ27XXX_I2C_BUS;
	gauge.chip = (CONFIG_FLIR_BQ27XXX_I2C_ADDR >> 1);

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret)
		log_err("BQ27: Battery fuelgauge not found\n");

	ret = bq_write_regpair(REG_CONTROL, CMD_DEVICE_TYPE);
	if (!ret)
		ret = bq_read_regpair(REG_CONTROL, &type);
	if (ret)
		return ret;

	gauge.type = type;
	if (type != TYPE_BQ27542 && type != TYPE_BQ27520) {
		log_err("BQ27: Unknown BQ27 typecode: 0x%04x (%u)\n", type, type);
		log_err("BQ27: Falling back to BQ27542\n");
		gauge.type = TYPE_BQ27542;
	}

	log_debug("%s(): type BQ27%x bus %d chip 0x%02x\n",
		  __func__, gauge.type, gauge.bus, gauge.chip);

	return ret;
}

// Reads are always done two registers at the time
static int bq_read_regpair(u8 reg, u16 *val)
{
	int ret;

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret) {
		log_err("BQ27: Battery fuelgauge not found\n");
		return ret;
	}
	*val = 0;
	ret = dm_i2c_read(dev, reg, (u8 *)val, 2);
	if (ret) {
		log_err("BQ27: i2c read error, returning %d\n", ret);
		return ret;
	}
	log_debug("%s(): reg 0x%x -> 0x%04x\n", __func__, reg, *val);

	return ret;
}

// Writes are always done two registers at the time
static int bq_write_regpair(u8 reg, u16 val)
{
	int ret;
	u8 buf[2];

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret) {
		log_err("BQ27: Battery fuelgauge not found\n");
		return ret;
	}
	buf[1] = (val >> 8) & 0xFF;
	buf[0] = val & 0xFF;
	ret = dm_i2c_write(dev, reg, &buf[0], 2);
	if (ret) {
		log_err("BQ27: i2c write error, returning %d\n", ret);
		return ret;
	}
	log_debug("%s(): reg 0x%x -> b0 0x%02x b1 0x%02x\n", __func__, reg, buf[0], buf[1]);

	return ret;
}

static int bq_write_buf(u8 reg, u8 *buf, int len)
{
	int ret;

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret) {
		log_err("BQ27: Battery fuelgauge not found\n");
		return ret;
	}
	ret = dm_i2c_write(dev, reg, buf, len);
	if (ret)
		log_err("BQ27: i2c write error, returning %d\n", ret);

	// Max reaction time to register writes
	mdelay(2);
	return ret;
}

static int bq_read_buf(u8 reg, u8 *buf, int len)
{
	int ret;

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret) {
		log_err("BQ27: Battery fuelgauge not found\n");
		return ret;
	}

	ret = dm_i2c_read(dev, reg, buf, len);
	if (ret)
		log_err("BQ27: i2c read error, returning %d\n", ret);

	return ret;
}

int bq27_read_cmd(const enum bq27_command cmd, u16 *rval)
{
	int ret = 0;
	int tmp = 0;

	if (!bq_is_initialized()) {
		ret = bq_init();
		if (ret)
			return ret;
	}

	switch (cmd) {
	case BQ_CONTROL_STATUS:
		ret = bq_write_regpair(REG_CONTROL, CMD_CONTROL_STATUS);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_DEVICE_TYPE:
		ret = bq_write_regpair(REG_CONTROL, CMD_DEVICE_TYPE);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_FW_VERSION:
		ret = bq_write_regpair(REG_CONTROL, CMD_FW_VERSION);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_DF_VERSION:
		if (gauge.type == TYPE_BQ27542)
			ret = bq_write_regpair(REG_CONTROL, BQ27542_CMD_DF_VERSION);
		else
			ret = bq_write_regpair(REG_CONTROL, BQ27520_CMD_DF_VERSION);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_RESET:
		ret = bq27_is_sealed(&tmp);
		if (tmp) {
			log_warning("BQ27: Cannot reset sealed fuel gauge\n");
		} else if (!ret) {
			ret = bq_write_regpair(REG_CONTROL, CMD_RESET);
			// Wait for FW boot, or consecutive calls will fail
			mdelay(200);
		}
		log_info("BQ27: RESET %s\n", ret || tmp ? "FAILED" : "OK");
		break;
	case BQ_CHEM_ID:
		ret = bq_write_regpair(REG_CONTROL, CMD_CHEM_ID);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_SET_HIBERNATE:
		ret = bq_write_regpair(REG_CONTROL, CMD_SET_HIBERNATE);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_CLEAR_HIBERNATE:
		ret = bq_write_regpair(REG_CONTROL, CMD_CLEAR_HIBERNATE);
		if (!ret)
			ret = bq_read_regpair(REG_CONTROL, rval);
		break;
	case BQ_TEMPERATURE:
		ret = bq_read_regpair(REG_TEMPERATURE, rval);
		break;
	case BQ_VOLTAGE:
		ret = bq_read_regpair(REG_VOLTAGE, rval);
		break;
	case BQ_STATE_OF_CHARGE:
		if (gauge.type == TYPE_BQ27542)
			ret = bq_read_regpair(BQ27542_REG_STATE_OF_CHARGE, rval);
		else
			ret = bq_read_regpair(BQ27520_REG_STATE_OF_CHARGE, rval);
		break;
	case BQ_DESIGN_CAPACITY:
		if (gauge.type == TYPE_BQ27542)
			ret = bq_read_regpair(BQ27542_REG_DESIGN_CAPACITY, rval);
		else
			ret = bq_read_regpair(BQ27520_REG_DESIGN_CAPACITY, rval);
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}

int bq27_unseal(void)
{
	int ret;
	u8 buf[4] = { 0x14, 0x04, 0x72, 0x36 };

	ret = bq_write_buf(0x00, &buf[0], 2);
	if (!ret)
		ret = bq_write_buf(0x00, &buf[2], 2);
	return ret;
}

static int check_control_bit(int bit, int *state)
{
	int ret;
	u16 status;

	if (!state)
		return -EINVAL;

	ret = bq27_read_cmd(BQ_CONTROL_STATUS, &status);
	if (!ret)
		*state = (status >> bit) & 1;
	return ret;
}

int bq27_is_sealed(int *seal_status)
{
	return check_control_bit(BIT_SS, seal_status);
}

int bq27_init_complete(int *ready)
{
	return check_control_bit(BIT_INITCOMP, ready);
}

#define ASCII_MIN (0x20)
#define ASCII_MAX (0x7a)
static void sanitize_maninfo_string(u8 *buf, int len)
{
	u8 val;

	if (buf[0] < ASCII_MIN || buf[0] > ASCII_MAX) {
		// First char is illegal, maninfo not properly set
		strncpy((char *)buf, "(unset)", len);
		return;
	}

	*(buf+(--len)) = 0;
	while (len--) {
		val = *(buf+len);
		if (val && (val < ASCII_MIN || val > ASCII_MAX))
			*(buf+len) = '.';
	}
}

int bq27_manufacturer_info(char **maninfo)
{
	int ret;
	int sealed;
	static u8 buf[BLOCK_DATA_MAXLEN];
	int len;

	*maninfo = NULL;
	ret = bq27_is_sealed(&sealed);
	if (ret)
		return ret;

	memset(buf, 0, sizeof(buf));
	if (sealed) {
		len = 1;
		buf[0] = 0x01;
		ret = bq_write_buf(REG_DATA_FLASH_BLOCK, buf, len);
		if (ret)
			return ret;

		buf[0] = 0x00;
		len = BLOCK_DATA_MAXLEN;
		if (!ret)
			ret = bq_read_buf(REG_BLOCK_DATA, buf, len);
		if (ret)
			return ret;
	} else {
		buf[0] = MAN_INFO_CLASS;
		len = 1;
		ret = bq_write_buf(REG_DATA_FLASH_CLASS, buf, len);
		if (ret)
			return ret;

		buf[0] = 0x00;
		ret = bq_write_buf(REG_DATA_FLASH_BLOCK, buf, len);
		if (ret)
			return ret;

		len = BLOCK_DATA_MAXLEN;
		if (!ret)
			ret = bq_read_buf(REG_BLOCK_DATA, buf, len);
		if (ret)
			return ret;
	}
	sanitize_maninfo_string(buf, sizeof(buf));
	*maninfo = (char *)buf;
	return ret;
}
