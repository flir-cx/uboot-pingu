// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 FLIR Systems.
 */
#include <log.h>
#include <i2c.h>
#include <dm/uclass.h>
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

static struct udevice *dev;
static struct {
	int type;
	int bus;
	int chip;
} gauge = {BQ_UNDEFINED, BQ_UNDEFINED, BQ_UNDEFINED};

static int bq_read_simple(u8 reg, u16 *val);
static int bq_write_simple(u8 reg, u16 val);

static int module_check_init(void)
{
	int ret;
	u16 type;

	if ((gauge.type != BQ_UNDEFINED) &&
	    (gauge.bus != BQ_UNDEFINED) &&
	    (gauge.chip != BQ_UNDEFINED))
		return 0;

	gauge.bus = CONFIG_FLIR_BQ27XXX_I2C_BUS;
	gauge.chip = (CONFIG_FLIR_BQ27XXX_I2C_ADDR >> 1);

	ret = i2c_get_chip_for_busnum(gauge.bus, gauge.chip, 1, &dev);
	if (ret)
		log_err("BQ27: Battery fuelgauge not found\n");

	ret = bq_write_simple(REG_CONTROL, CMD_DEVICE_TYPE);
	if (!ret)
		ret = bq_read_simple(REG_CONTROL, &type);
	if (ret)
		return ret;

	gauge.type = type;
	if ((type != TYPE_BQ27542) && (type != TYPE_BQ27520)) {
		log_err("BQ27: Unkown BQ27 typecode: 0x%04x (%u)\n", type, type);
		log_err("BQ27: Falling back to BQ27542\n");
		gauge.type = TYPE_BQ27542;
	}

	log_debug("%s(): type BQ27%x bus %d chip 0x%02x\n",
		  __func__, gauge.type, gauge.bus, gauge.chip);

	return ret;
}

// Reads are always done in 2-byte chunks
static int bq_read_simple(u8 reg, u16 *val)
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
		log_err("BQ27: dm_i2c_read() Read error. Returning %d\n", ret);
		return ret;
	}
	log_debug("%s(): reg 0x%x -> 0x%04x\n", __func__, reg, *val);

	return ret;
}

// Writes are always done in 2-byte chunks
static int bq_write_simple(u8 reg, u16 val)
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
		log_err("BQ27: dm_i2c_read() Read error. Returning %d\n", ret);
		return ret;
	}
	log_debug("%s(): reg 0x%x -> b0 0x%02x b1 0x%02x\n", __func__, reg, buf[0], buf[1]);

	return ret;
}

int bq27_read(const enum bq27_command cmd, u16 *rval)
{
	int ret = 0;

	ret = module_check_init();
	if (ret)
		return ret;

	switch (cmd) {
	case BQ_CONTROL_STATUS:
		ret = bq_write_simple(REG_CONTROL, CMD_CONTROL_STATUS);
		if (!ret)
			ret = bq_read_simple(REG_CONTROL, rval);
		break;
	case BQ_DEVICE_TYPE:
		ret = bq_write_simple(REG_CONTROL, CMD_DEVICE_TYPE);
		if (!ret)
			ret = bq_read_simple(REG_CONTROL, rval);
		break;
	case BQ_FW_VERSION:
		ret = bq_write_simple(REG_CONTROL, CMD_FW_VERSION);
		if (!ret)
			ret = bq_read_simple(REG_CONTROL, rval);
		break;
	case BQ_DF_VERSION:
		log_info("%s: type is 0x%04x\n", __func__, gauge.type);
		if (gauge.type == TYPE_BQ27542)
			ret = bq_write_simple(REG_CONTROL, BQ27542_CMD_DF_VERSION);
		else
			ret = bq_write_simple(REG_CONTROL, BQ27520_CMD_DF_VERSION);
		if (!ret)
			ret = bq_read_simple(REG_CONTROL, rval);
		break;
	case BQ_CHEM_ID:
		ret = bq_write_simple(REG_CONTROL, CMD_CHEM_ID);
		if (!ret)
			ret = bq_read_simple(REG_CONTROL, rval);
		break;
	case BQ_VOLTAGE:
		ret = bq_read_simple(REG_VOLTAGE, rval);
		break;
	case BQ_STATE_OF_CHARGE:
		if (gauge.type == TYPE_BQ27542)
			ret = bq_read_simple(BQ27542_REG_STATE_OF_CHARGE, rval);
		else
			ret = bq_read_simple(BQ27520_REG_STATE_OF_CHARGE, rval);
		break;
	case BQ_DESIGN_CAPACITY:
		if (gauge.type == TYPE_BQ27542)
			ret = bq_read_simple(BQ27542_REG_DESIGN_CAPACITY, rval);
		else
			ret = bq_read_simple(BQ27520_REG_DESIGN_CAPACITY, rval);
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}
