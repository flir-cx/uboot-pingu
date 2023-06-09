// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>

#include "lc709203.h"

static int get_fuelgauge_device(struct udevice **dev)
{
	static struct udevice *fuelgauge_dev;
	int ret = 0;

	if (!fuelgauge_dev) {
		ret = i2c_get_chip_for_busnum(FUELGAUGE_I2C_BUS, FUELGAUGE_I2C_ADDR, 1, &fuelgauge_dev);
		if (ret)
			printf("Cannot find fuelgauge LC709203: %d\n", ret);
	}

	*dev = fuelgauge_dev;

	return ret;
}

static u8 crc8(const void *vptr, int len)
{
	const u8 *data = vptr;
	unsigned int crc = 0;
	int i, j;

	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for (i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}

	return (u8)(crc >> 8);
}

int fuelgauge_write_reg(uint reg, u8 lb, u8 hb)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;
	u8 crc;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	/* Calculate crc */
	buf[0] = FUELGAUGE_I2C_ADDR << 1;
	buf[1] = reg;
	buf[2] = lb;
	buf[3] = hb;
	crc = crc8(buf, 4);

	buf[0] = lb;
	buf[1] = hb;
	buf[2] = crc;
	dm_i2c_write(dev, reg, buf, 3);

	return 0;
}

static int fuelguage_get_type(void)
{
	u8 buf[4];
	int ret;
	struct udevice *dev;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, IC_VERSION, buf, 2);

	if (ret)
		return ret;

	return *(u16 *)buf == 0x301;
}

int fuelgauge_get_state_of_charge(u16 *soc)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, 0xd, buf, 2);
	if (!ret)
		*soc = (buf[1] << 8) | buf[0];

	return ret;
}

int fuelgauge_get_battery_voltage(u16 *voltage)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, 0x9, buf, 2);
	if (!ret)
		*voltage = (buf[1] << 8) | buf[0];

	return ret;
}

int fuelgauge_sleep(void)
{
	// Only lc709203f should be put to sleep, lc709204f should stay in operational mode
	if (fuelguage_get_type() == LC709204F)
		return 0;

	return fuelgauge_write_reg(IC_POWER_MODE_REG, FUELGAUGE_SLEEP_MODE, 0);
}

int fuelgauge_operational(void)
{
	/* First write is to enable i2c communication with the fuelgauge */
	int ret;

	ret = fuelgauge_write_reg(IC_POWER_MODE_REG, OPERATIONAL_MODE, 0);
	if (ret)
		return ret;

	udelay(1000);

	return fuelgauge_write_reg(IC_POWER_MODE_REG, OPERATIONAL_MODE, 0);
}

int fuelgauge_thermistor_mode(void)
{
	return fuelgauge_write_reg(STATUS_BIT_REG, THERMISTOR_MODE, 0);
}

/*
 * The battery can either be:
 *
 *  - Missing
 *  - Inserted but not initialized, or
 *  - Inserted and initialized
 *
 * To differentiate between these cases we read the content of a
 * register that is non-zero if initialized and zero if not. If we
 * cannot read the register at all we assume that the battery is
 * missing.
 */
int fuelgauge_check_battery_insertion(void)
{
	struct udevice *dev;
	u8 buf;
	int ret;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return BATTERY_NONE;
	}

	ret = dm_i2c_read(dev, LC709204_APA, &buf, 1);
	if (ret) {
		printf("Failed to read fuelgauge register\n");
		return BATTERY_NONE;
	}

	if (buf)
		return BATTERY_PLUGGED_IN;

	printf("Battery inserted!\n");

	return BATTERY_INSERTED;
}

int fuelgauge_init(void)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;
	int type;
	int battery_inserted;

	ret = get_fuelgauge_device(&dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	/* Power on fuelgauge from standby */
	fuelgauge_operational();

	/* Thermistor mode: fuelgauge measures the attached resistor */
	fuelgauge_thermistor_mode();

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
	/* Set battery profile */
	fuelgauge_write_reg(CHANGE_OF_THE_PARAMETER_REG, 0x00, 0x00);
	/* Set APA value */
	fuelgauge_write_reg(LC709204_APA, 0x29, 0x29);
	/* Set ITE Offset, will scale RSOC to reach 0% when 3.2V */
	fuelgauge_write_reg(LC709204_ITE_OFFSET, 0x15, 0x00);
	/* Set CHG_TERM_CURR (taper current in 0.01C), 0.03C => 54mA with 1800mA battery */
	fuelgauge_write_reg(LC709204_CHG_TERM_CURR, 0x03, 0x00);
	/* Empty Cell Voltage. 0 will disable ITE offset update. */
	fuelgauge_write_reg(LC709204_EMPTY_VOLT, 0x00, 0x00);
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201))
	/* Set battery profile */
	type = fuelguage_get_type();
	fuelgauge_write_reg(CHANGE_OF_THE_PARAMETER_REG, type, 0x00);
	/* Set APA value */
	fuelgauge_write_reg(LC709204_APA, 0x2d, 0x2d);
	/* Set ITE Offset, will scale RSOC to reach 0% when 3.2V */
	fuelgauge_write_reg(LC709204_ITE_OFFSET, 0x15, 0x00);
	/* Set CHG_TERM_CURR (taper current in 0.01C), 0.03C => 54mA with 1800mA battery */
	fuelgauge_write_reg(LC709204_CHG_TERM_CURR, 0x03, 0x00);
	/* Empty Cell Voltage. 0 will disable ITE offset update. */
	fuelgauge_write_reg(LC709204_EMPTY_VOLT, 0x00, 0x00);
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
	/* Set battery profile */
	fuelgauge_write_reg(CHANGE_OF_THE_PARAMETER_REG, 0x02, 0x00);
	/* Set APA value */
	fuelgauge_write_reg(LC709204_APA, 0x06, 0x06);
	/* Set ITE Offset, will scale RSOC to reach 0% when 3.2V */
	fuelgauge_write_reg(LC709204_ITE_OFFSET, 0x00, 0x00);
	/* Set CHG_TERM_CURR (taper current in 0.01C), 0.03C => 54mA with 1800mA battery */
	fuelgauge_write_reg(LC709204_CHG_TERM_CURR, 0x03, 0x00);
	/* Empty Cell Voltage. 0 will disable ITE offset update. */
	fuelgauge_write_reg(LC709204_EMPTY_VOLT, 0x00, 0x00);
#endif

	return 0;
}
