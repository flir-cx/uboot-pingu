// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#ifndef _LC709203_
#define _LC709203_

#include <linux/types.h>

#define FUELGAUGE_I2C_ADDR 0xb
#define FUELGAUGE_I2C_BUS 5
#define CHANGE_OF_THE_PARAMETER_REG 0x12
#define BATTERY_PROFILE_ONE 0x01
#define BATTERY_PROFILE_ZERO 0x00

#define IC_POWER_MODE_REG 0x15
#define STATUS_BIT_REG	0x16
#define IC_VERSION 0x1a
#define THERMISTOR_MODE 0x1
#define OPERATIONAL_MODE 0x1
#define FUELGAUGE_SLEEP_MODE 0x2

#define LC709204_APA 0x0b
#define LC709204_CHG_TERM_CURR 0x1c
#define LC709204_EMPTY_VOLT 0x1d
#define LC709204_ITE_OFFSET 0x1e

#define LC709203F 1
#define LC709204F 0

enum fuelgauge_battery_status {BATTERY_NONE, BATTERY_INSERTED, BATTERY_PLUGGED_IN};

int fuelgauge_init(void);

int fuelgauge_get_state_of_charge(u16 *soc);
int fuelgauge_get_battery_voltage(u16 *voltage);
int fuelgauge_sleep(void);
int fuelgauge_operational(void);
int fuelgauge_check_battery_insertion(void);

int fuelgauge_thermistor_mode(void);
int fuelgauge_write_reg(uint reg, u8 lb, u8 hb);

#endif // _LC709203_
