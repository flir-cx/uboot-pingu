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
#include <video.h>
#include <linux/delay.h>
#include <video_console.h>
#include <splash.h>

#include "../common/display_utils.h"

DECLARE_GLOBAL_DATA_PTR;

#define CSI "\x1b["

#define TRIG_GPIO   IMX_GPIO_NR(3, 12)

#define BTN_GPIO_PAD_CTRL	(PAD_CTL_IBE_ENABLE | PAD_CTL_PUS_UP)

#define TRIG_ON 0
#define TRIG_OFF 1
#define POLL_TIME 10

#define OK 0
#define FAIL 1

static iomux_cfg_t const btn_trig_pad[] = {
	MX7ULP_PAD_PTC12__PTC12 | MUX_PAD_CTRL(BTN_GPIO_PAD_CTRL),
};

// timeout in ms
int trigger_wait_until(int btn_state, int mtimeout)
{
    int t = 0;
    int ret = FAIL;

    while(gpio_get_value(TRIG_GPIO) != btn_state && t < mtimeout)
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
    int ret = FAIL;
    ret = trigger_wait_until(TRIG_ON, mtimeout);

    if(ret == OK)
        ret = trigger_wait_until(TRIG_OFF, mtimeout);

    return ret;
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

int get_serial_number(uint8_t *buf)
{
    struct udevice *dev;
    int ret = i2c_get_chip_for_busnum(6, 0x57, 1, &dev); //get eprom
    if (ret) {
		printf("Can not find eeprom: %d\n", ret);
		return ret;
	}

    ret = dm_i2c_read(dev, 0x24, buf, 10);

    return ret;
}

int check_recovery_sequence(void)
{
    int ret = -1;

	display_set_text_color();

    display_print_string(" *");
    //wait for 0
    ret = trigger_wait_until(TRIG_OFF, 3000);

    if(ret == OK)
    {
        // sequence 1
        printf("sequence 1 start \n");

        if(seq(5) == FAIL)
            return FAIL;

        if(trigger_wait_until(TRIG_ON, 1000) == OK)
            return FAIL; // dont press again.

        printf("sequence 1 ok\n");
        display_print_string("**");

        // sequence 2
        printf("sequence 2 start \n");

        if(seq(6) == FAIL)
            return FAIL;

        printf("sequence 2 ok\n");

        if(trigger_wait_until(TRIG_ON, 2000) == OK)
            return FAIL; // dont press again.

        char buf[24];
        buf[0] = '\0';
        char buf2[24];
        get_serial_number((uint8_t *) buf);

        if(buf[0] == 0xff || strlen(buf) == 0)
            display_print_string("Recovery *");//No Serial
        else
        {
            snprintf(buf2, 19, "Recovery %s", buf);
            display_print_string(buf2);
        }
    }

    printf("get_recovery_sequence: %d\n", ret);
    return ret;
}

int do_recoverytrigger(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    int trig = 0;
    int res = FAIL;

    printf("Check recovery\n");

    pcc_clock_enable(PER_CLK_PCTLC, true);

    mx7ulp_iomux_setup_multiple_pads(btn_trig_pad, ARRAY_SIZE(btn_trig_pad));

    gpio_request(TRIG_GPIO, "trig_gpio");

	trig = gpio_direction_input(TRIG_GPIO);

    if(trig == -1)
    {
        return FAIL;
    }

	trig = gpio_get_value(TRIG_GPIO);

    if(trig == 0)
    {
        // Check recovery sequence
        printf("Trigger on, check recovery.\n");
        res = check_recovery_sequence();

        if(res == FAIL)
        {
            display_print_string("                      "); //clear screen
        }
    }

    return res;
}

U_BOOT_CMD(
    recoverytrigger,	2,	1,	do_recoverytrigger,
    "do_recoverytrigger",
    "do_recoverytrigger"
);
