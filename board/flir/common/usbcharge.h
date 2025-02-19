/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __USBCHARGE_H
#define __USBCHARGE_H
int usb_charge_setup(void);
void set_boot_logo(void);
void get_pmic_regs(unsigned char *event_a, unsigned char *status_a);
bool get_gauge_state(void);
int get_battery_level(void);

/**
 * battery_overheat() - Temp-check function
 * Override this weak function to add high-temp functionality.
 *
 * Return: true if temp too high to boot
 */
bool battery_overheat(void);

/**
 * supply_voltage_present() - Detect non-battery power supply
 * @voltage: measured voltage at the fuelgauge
 *
 * Override this weak function to check for external power.
 * Compare measured voltage to a reasonable limit and indicate
 * that some kind of supply power is present.
 *
 * Return: true if voltage is high enough to run
 */
bool supply_voltage_present(int voltage);

enum BOOT_STATES {
	NORMAL_BOOT = 0,
	LOW_BATTERY,
	NO_BATTERY,
	USB_CHARGE,
	HOT_BATTERY,
};

#define FAKE_BATTERY_LEVEL 50

#endif
