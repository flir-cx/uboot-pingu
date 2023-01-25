/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __USBCHARGE_H
#define __USBCHARGE_H
int usb_charge_setup(void);
void set_boot_logo(void);
void get_pmic_regs(unsigned char *event_a, unsigned char *status_a);
bool get_gauge_state(void);
int get_battery_level(bool do_print);

enum BOOT_STATES {
	NORMAL_BOOT = 0,
	LOW_BATTERY,
	NO_BATTERY,
	USB_CHARGE,
};

#define FAKE_BATTERY_LEVEL 50

#endif
