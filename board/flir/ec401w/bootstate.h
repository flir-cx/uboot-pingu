
#ifndef _BOOTSTATE_H
#define _BOOTSTATE_H

void set_boot_logo(void);

int boot_state_init(void);

int get_battery_state_of_charge(int *soc);

#endif