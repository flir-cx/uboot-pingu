#ifndef _LED_UTILS_H
#define _LED_UTILS_H

#define CHARGING     1
#define NOT_CHARGING 0

void leds_battery_solid(int soc);
void leds_charge(int soc);
void leds_critical(void);
void leds_battery_pulse(int soc);
void leds_off(void);
void leds_all_blink_slow(void);
void leds_all_blink_fast(void);
void leds_boot(void);

#endif
