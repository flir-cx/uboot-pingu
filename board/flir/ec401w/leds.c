#include <common.h>
#include <asm/io.h>
#include <dm.h>
#include <console.h>
#include <command.h>
#include <leds.h>

typedef enum {
	LEDS_CHG_LOW,
	LEDS_CHG_MED,
	LEDS_CHG_HIGH,
	LEDS_CHG_FULL,
    LEDS_CRITICAL,
	LEDS_BLINK_SLOW,
	LEDS_BLINK_FAST,
	LEDS_BLINK_BOOT,
    LEDS_LOW,
    LEDS_MED,
    LEDS_HIGH,
	LEDS_OFF
} leds_state_t;

#define CHG_LEVEL_FULL 95
#define CHG_LEVEL_HIGH 80
#define CHG_LEVEL_MED 50
#define CHG_LEVEL_LOW 3

#define CHARGING     1
#define NOT_CHARGING 0

static leds_state_t leds_state = LEDS_OFF;

void leds_on(int soc, int chg)
{
	leds_state_t new_state;

    if(chg == CHARGING)
    {
        if (soc >= CHG_LEVEL_FULL)
			new_state = LEDS_CHG_FULL;
        else if (soc >= CHG_LEVEL_HIGH)
            new_state = LEDS_CHG_HIGH;
        else if (soc >= CHG_LEVEL_MED)
            new_state = LEDS_CHG_MED;
        else
            new_state = LEDS_CHG_LOW;
    }
    else
    {
        if (soc >= CHG_LEVEL_HIGH)
            new_state = LEDS_HIGH;
        else if (soc >= CHG_LEVEL_MED)
            new_state = LEDS_MED;
        else if (soc >= CHG_LEVEL_LOW)
            new_state = LEDS_LOW;
        else
            new_state = LEDS_CRITICAL;
    }

	if(leds_state == new_state)
		return;

	printf("leds set soc:%d chg:%d\n", soc, chg);

	switch(new_state)
	{
		case LEDS_CHG_FULL:
			run_command("led 1 on", 0);
			run_command("led 2 on", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			leds_state=LEDS_CHG_FULL;
			break;
		case LEDS_CHG_HIGH:
			run_command("led 1 on", 0);
			run_command("led 1 blink 500", 0);
			run_command("led 2 on", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			leds_state=LEDS_CHG_HIGH;
			break;
		case LEDS_CHG_MED:
			run_command("led 1 off", 0);
			run_command("led 2 on", 0);
			run_command("led 2 blink 500", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			leds_state=LEDS_CHG_MED;
			break;
		case LEDS_CHG_LOW:
			run_command("led 1 off", 0);
			run_command("led 2 off", 0);
			run_command("led 3 off", 0);
			run_command("led 3 blink 500", 0);
			run_command("led r off", 0);
			leds_state=LEDS_CHG_LOW;
			break;

		case LEDS_HIGH:
			run_command("led 1 on", 0);
			run_command("led 2 on", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			break;
		case LEDS_MED:
			run_command("led 1 off", 0);
			run_command("led 2 on", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			break;
		case LEDS_LOW:
			run_command("led 1 off", 0);
			run_command("led 2 off", 0);
			run_command("led 3 on", 0);
			run_command("led r off", 0);
			break;
		case LEDS_CRITICAL:
			run_command("led 1 off", 0);
			run_command("led 2 off", 0);
			run_command("led 3 off", 0);
			run_command("led r blink 250", 0); //red
		case LEDS_OFF:
			leds_off();
			break;
		default:
			break;
	}
	return;
}

void leds_off(void)
{
	// Fade all
	run_command("led p 2 500", 0);
	run_command("led r off", 0);
	leds_state = LEDS_OFF;
	return;
}

void leds_all_blink_slow(void)
{
	run_command("led r off", 0);
	run_command("led p 3 1000", 0);
	leds_state = LEDS_BLINK_SLOW;
}

void leds_all_blink_fast(void)
{
	run_command("led r off", 0);
	run_command("led p 3 500", 0);
	leds_state = LEDS_BLINK_FAST;
}

void leds_boot(void)
{
	run_command("led r off", 0);
	run_command("led p 0 600", 0);
	leds_state = LEDS_BLINK_BOOT;
}