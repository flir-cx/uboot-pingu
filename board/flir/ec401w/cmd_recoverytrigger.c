/* * Copyright (C) 2019 FLIR Systems.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Recovery sequence
 * 
 *  1. Hold trigger and power button while booting, release when screen lights up.
 *  2. Press trigger button 5 times
 *  3. Wait until 2 stars appears on screen
 *  4. Press trigger button 6 times
 *  5. Camera goes to recovery.
 * 
 */


#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <command.h>
#include <stdio_dev.h>
#include <dm.h>
#include <i2c.h>
#include <asm/arch/pcc.h>
#include <linux/delay.h>
#include "leds.h"
#include "bootstate.h"
#include "lc709203.h"

#include "../common/pf1550.h"

DECLARE_GLOBAL_DATA_PTR;

#define PWR_GPIO   IMX_GPIO_NR(3, 13)
#define BTN_GPIO_PAD_CTRL	(PAD_CTL_IBE_ENABLE | PAD_CTL_PUS_UP)

static iomux_cfg_t const btn_pwr_pad[] = {
	MX7ULP_PAD_PTC13__PTC13 | MUX_PAD_CTRL(BTN_GPIO_PAD_CTRL),
};

/* Total time has to be less than 16 seconds,
 * since hw reset kicks in.
 */
// Circuit bootup is around 1.5 seconds
#define SEQ_BATTERY_STATUS_START_MS 2500	/* 4s  */
#define SEQ_DISPLAY_SOC_MS          6000	/* 10s */
#define SEQ_SLOW_FDEFAULT_MS        2000	/* 12s */
#define SEQ_FAST_FDEFAULT_MS        2000	/* 14s */
#define SEQ_END_MS                  2000	/* 16s */

#define TRIG_ON 0
#define TRIG_OFF 1
#define POLL_TIME 10

#define OK 0
#define FAIL 1

// timeout in ms
int trigger_wait_until(int btn_state, int mtimeout)
{
    int t = 0;
    int ret = FAIL;

    while(gpio_get_value(PWR_GPIO) != btn_state && t < mtimeout)
    {
        mdelay(POLL_TIME);
        t += POLL_TIME;
    }

    if(t < mtimeout)
    {
        ret = OK;
    }
    else
        ret = FAIL;

    return ret;
}

int get_press(int mtimeout)
{
    if(trigger_wait_until(TRIG_ON, mtimeout) == OK)
        return trigger_wait_until(TRIG_OFF, mtimeout);

    return FAIL;
}


int seq(int no)
{
    if(get_press(2000) == OK)
    {
        for(int i=0; i<no-1;i++)
        {
            if(get_press(700) != OK)
               return FAIL;
        }
    }
    else
        return FAIL;

    return OK;
}



int check_recovery_sequence(void)
{
    // sequence 1
    printf("sequence 1 start \n");

    if(seq(5) == FAIL)
        return FAIL;

    if(trigger_wait_until(TRIG_ON, 1000) == OK)
        return FAIL; // dont press again.

    printf("sequence 1 ok\n");

	// One led is on
    leds_battery_solid(20);

    // sequence 2
    printf("sequence 2 start \n");

    if(seq(6) == FAIL)
        return FAIL;

    printf("sequence 2 ok\n");

    if(trigger_wait_until(TRIG_ON, 2000) == OK)
        return FAIL; // dont press again.

    leds_boot();

    return OK;
}

int check_button_sequence(void)
{
    int soc;

    printf("Check button sequence\n");

    if(trigger_wait_until(TRIG_OFF, SEQ_BATTERY_STATUS_START_MS) == OK)
        return FAIL;

    // Display SOC
    get_battery_state_of_charge(&soc);
	leds_battery_pulse(soc);

    printf("Button sequence 1\n");

	mdelay(SEQ_DISPLAY_SOC_MS);
    if(gpio_get_value(PWR_GPIO) == TRIG_OFF)
        power_off();

    printf("Button sequence 2\n");

    //factory default sequence 1
    leds_all_blink_slow();

    if(trigger_wait_until(TRIG_OFF, SEQ_SLOW_FDEFAULT_MS) == OK)
    {
        // Only go to recovery if cable is inserted
        if(!get_usb_cable_state() || check_recovery_sequence() == FAIL)
        {
            leds_off();
            power_off();
        }
        else
        {
            return OK;
        }
    }

    printf("Button sequence 3\n");

    //factory default sequence 2
    leds_all_blink_fast();

    if(trigger_wait_until(TRIG_OFF, SEQ_FAST_FDEFAULT_MS) == OK)
    {
        leds_off();
        power_off();
    }

    printf("Button sequence 4\n");

    leds_battery_solid(100);

    if(trigger_wait_until(TRIG_OFF, SEQ_END_MS) == OK)
    {
        printf("Factory default\n");
        // User release button while steady leds
        // inject factory default here
        env_set("factorydefault_bootarg", "factorydefault");
		leds_boot();
	}
	else
	{
		leds_off();
		power_off();
	}

    return FAIL;
}

static int do_recoverytrigger(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    int pwr_btn = 0;

    env_set("factorydefault_bootarg", "");

    pcc_clock_enable(PER_CLK_PCTLC, true);

    mx7ulp_iomux_setup_multiple_pads(btn_pwr_pad, ARRAY_SIZE(btn_pwr_pad));

    gpio_request(PWR_GPIO, "pwr_gpio");

	pwr_btn = gpio_direction_input(PWR_GPIO);

    if(pwr_btn == -1)
    {
        return FAIL;
    }

    pwr_btn = gpio_get_value(PWR_GPIO);

    if(pwr_btn == 0)
    {

        if(check_button_sequence() == OK)
        {
            return OK;
        }
    }

    return FAIL;
}

U_BOOT_CMD(
    recoverytrigger,	2,	1,	do_recoverytrigger,
    "do_recoverytrigger",
    "do_recoverytrigger"
);
