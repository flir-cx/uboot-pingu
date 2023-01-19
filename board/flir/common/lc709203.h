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
#define LC709204_ChgTermCurr 0x1c
#define LC709204_Empty_Volt 0x1d
#define LC709204_ITE_Offset 0x1e

#define LC709203F 1
#define LC709204F 0

int fuelgauge_init(void);

int fuelgauge_get_state_of_charge(int *soc);
int fuelgauge_sleep(void);
int fuelgauge_operational(void);

int fuelgauge_battery_profile_one(void);
int fuelgauge_thermistor_mode(void);
int fuelgauge_write_reg(uint reg, u8 lb, u8 hb);

#endif // _LC709203_
