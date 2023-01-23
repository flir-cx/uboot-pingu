// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <common.h>
#include <asm/io.h>
#include <dm.h>
#include <console.h>
#include <command.h>
#include "led_utils.h"

#define CHG_LEVEL_FULL 95
#define CHG_LEVEL_HIGH 75
#define CHG_LEVEL_LOW 35
#define CHG_LEVEL_CRITICAL 10

void leds_battery_solid(int soc)
{
	run_command("led r off", 0);

	if (soc >= CHG_LEVEL_HIGH)
		run_command("led p 5 3", 0);
	else if (soc >= CHG_LEVEL_LOW)
		run_command("led p 5 2", 0);
	else
		run_command("led p 5 1", 0);
}

void leds_charge(int soc)
{
	run_command("led r off", 0);

	if (soc >= CHG_LEVEL_FULL)
		leds_battery_solid(soc);
	else if (soc >= CHG_LEVEL_HIGH)
		run_command("led p 4 3", 0);
	else if (soc >= CHG_LEVEL_LOW)
		run_command("led p 4 2", 0);
	else
		run_command("led p 4 1", 0);
}

void leds_critical(void)
{
	run_command("led 1 off", 0);
	run_command("led 2 off", 0);
	run_command("led 3 off", 0);
	run_command("led r blink 250", 0); // Blinking red
}

void leds_off(void)
{
	// Fade all
	run_command("led r off", 0);
	run_command("led p 2 500", 0);
}

void leds_all_blink_slow(void)
{
	run_command("led r off", 0);
	run_command("led p 3 1000", 0);
}

void leds_all_blink_fast(void)
{
	run_command("led r off", 0);
	run_command("led p 3 500", 0);
}

void leds_battery_pulse(int soc)
{
	run_command("led r off", 0);

	if (soc >= CHG_LEVEL_HIGH)
		run_command("led p 1 3", 0);
	else if (soc >= CHG_LEVEL_LOW)
		run_command("led p 1 2", 0);
	else
		run_command("led p 1 1", 0);
}

void leds_boot(void)
{
	run_command("led r off", 0);
	run_command("led p 0", 0); /* 600 is a dummy param */
}
