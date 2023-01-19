
/*
 * Copyright 2018 FLIR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>

#include "lc709203.h"

static uint8_t crc8(const void* vptr, int len) {
	const uint8_t *data = vptr;
	unsigned crc = 0;
	int i, j;

	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for(i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}

	return (uint8_t)(crc >> 8);
}

int fuelgauge_write_reg(uint reg, u8 lb, u8 hb)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;
	uint8_t crc;

	ret = i2c_get_chip_for_busnum(FUELGAUGE_I2C_BUS, FUELGAUGE_I2C_ADDR, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	/* Calculate crc */
	buf[0] = FUELGAUGE_I2C_ADDR << 1;
	buf[1] = reg;
	buf[2] = lb;
	buf[3] = hb;
	crc = crc8(buf,4);

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

	ret = i2c_get_chip_for_busnum(5, 0xb, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, IC_VERSION, buf, 2);

	if(ret)
		return ret;

	return *(u16 *)buf == 0x301;
}

int fuelgauge_get_state_of_charge(int *soc)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;

	ret = i2c_get_chip_for_busnum(5, 0xb, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, 0xd, buf, 2);
	if(!ret)
		*soc = *(u16*)buf;

	return ret;
}

int fuelgauge_sleep(void)
{
	// Only lc709203f should be put to sleep, lc709204f should stay in operational mode
	if(fuelguage_get_type() == LC709204F)
		return 0;

	return fuelgauge_write_reg(IC_POWER_MODE_REG, FUELGAUGE_SLEEP_MODE, 0);
}

int fuelgauge_operational(void)
{
	/* First write is to enable i2c communication with the fuelgauge */
	int ret;
	ret = fuelgauge_write_reg(IC_POWER_MODE_REG, OPERATIONAL_MODE, 0);
	if(ret)
		return ret;

	udelay(1000);

	return fuelgauge_write_reg(IC_POWER_MODE_REG, OPERATIONAL_MODE, 0);
}

int fuelgauge_thermistor_mode(void)
{
	return fuelgauge_write_reg(STATUS_BIT_REG, THERMISTOR_MODE, 0);
}

int fuelgauge_battery_profile(int param)
{
	return fuelgauge_write_reg(CHANGE_OF_THE_PARAMETER_REG, param?BATTERY_PROFILE_ONE:BATTERY_PROFILE_ZERO, 0);
}

int fuelgauge_init(void)
{
	struct udevice *dev;
	u8 buf[4];
	int ret;
	int type;

	ret = i2c_get_chip_for_busnum(5, 0xb, 1, &dev);
	if (ret) {
		printf("Cannot find fuelgauge LC709203: %d\n", ret);
		return ret;
	}

	/* Power on fuelgauge from standby */
	fuelgauge_operational();

	/* Thermistor mode: fuelgauge measures the attached resistor */
	fuelgauge_thermistor_mode();

	// param should be 1 for lc709203f and 0 for lc709203f
	type = fuelguage_get_type();
	printf("found %s\n", type ? "LC709203" : "LC709204");

	/* Check if battery profile already is selected
	* Every write to this register will recalibrate the fuelgauge,
	* which we only want to do once */
	ret = dm_i2c_read(dev, CHANGE_OF_THE_PARAMETER_REG, buf, 2);
	if (ret == 0 && *(u16 *)buf != type)
	{
		/* Select battery profile */
		fuelgauge_battery_profile(type);
	}

	if (ret == 0 && type == LC709204F)
	{
		/* Setup as LC709204 */
#ifdef CONFIG_TARGET_MX7ULP_EC401W
		fuelgauge_write_reg(LC709204_APA, 0x29, 0x29);		// set APA value 2929
#elif defined(CONFIG_TARGET_MX7ULP_EC201) || defined(CONFIG_TARGET_MX7ULP_EC302)
		fuelgauge_write_reg(LC709204_APA, 0x2d, 0x2d);		// set APA value 2d2d
#endif
		fuelgauge_write_reg(LC709204_ChgTermCurr, 0x03, 0);	// set ChgTermCurr (taper current in 0.01C), 0.03C => 54mA with 1800mA battery
		fuelgauge_write_reg(LC709204_Empty_Volt, 0x00, 0);	// Empty Cell Voltage. 0 will disable ITE offset update.
		fuelgauge_write_reg(LC709204_ITE_Offset, 0x15, 0);	// set ITE Offset, will scale RSOC to reach 0% when 3.2V
	}

	return 0;
}
