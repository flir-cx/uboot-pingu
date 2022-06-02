#ifndef _LEDS_H
#define _LEDS_H

#define CHARGING     1
#define NOT_CHARGING 0

void leds_on(int soc, int chg);
void leds_off(void);
void leds_all_blink_slow(void);
void leds_all_blink_fast(void);
void leds_boot(void);

#endif