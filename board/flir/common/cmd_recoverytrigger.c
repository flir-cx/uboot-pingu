// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
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

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201) || CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
#include "display_utils.h"
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
#include "led_utils.h"
#include "pf1550.h"
#include "lc709203.h"
#endif

DECLARE_GLOBAL_DATA_PTR;

#define TRIG_ON 0
#define TRIG_OFF 1
#define POLL_TIME 10

#define OK 0
#define FAIL 1

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201))
#define TRIGGER_PIN "GPIO3_12"
static iomux_cfg_t const btn_trig_pad[] = {
	MX7ULP_PAD_PTC12__PTC12 | MUX_PAD_CTRL(PAD_CTL_IBE_ENABLE | PAD_CTL_PUS_UP),
};
#define GPIO_PER_CLK PER_CLK_PCTLC
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
#define TRIGGER_PIN "GPIO6_0"
static iomux_cfg_t const btn_trig_pad[] = {
	MX7ULP_PAD_PTF0__PTF0 | MUX_PAD_CTRL(PAD_CTL_IBE_ENABLE | PAD_CTL_PUS_UP),
};
#define GPIO_PER_CLK PER_CLK_PCTLF
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
#define TRIGGER_PIN "GPIO3_13"
static iomux_cfg_t const btn_trig_pad[] = {
	MX7ULP_PAD_PTC13__PTC13 | MUX_PAD_CTRL(PAD_CTL_IBE_ENABLE | PAD_CTL_PUS_UP),
};
#define GPIO_PER_CLK PER_CLK_PCTLC

/* Total time has to be less than 16 seconds,
 * since hw reset kicks in.
 * Circuit bootup is around 1.5 seconds
 */
#define SEQ_BATTERY_STATUS_START_MS 2500	/* 4s  */
#define SEQ_DISPLAY_SOC_MS          6000	/* 10s */
#define SEQ_SLOW_FDEFAULT_MS        2000	/* 12s */
#define SEQ_FAST_FDEFAULT_MS        2000	/* 14s */
#define SEQ_END_MS                  2000	/* 16s */
#endif

struct gpio_desc trigger_gpio_desc;

int get_button_state(void)
{
	return dm_gpio_get_value(&trigger_gpio_desc);
}

int trigger_wait_until(int btn_state, int mtimeout)
{
	int t = 0;

	while (get_button_state() != btn_state && t < mtimeout) {
		mdelay(POLL_TIME);
		t += POLL_TIME;
	}

	if (t < mtimeout)
		return OK;

	return FAIL;
}

int get_press(int mtimeout)
{
	if (trigger_wait_until(TRIG_ON, mtimeout) == OK)
		return trigger_wait_until(TRIG_OFF, mtimeout);

	return FAIL;
}

int seq(int no)
{
	if (get_press(2000) != OK)
		return FAIL;

	for (int i = 0; i < no - 1; i++) {
		if (get_press(700) != OK)
			return FAIL;
	}

	return OK;
}

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201) || CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
static int get_serial_number(uint8_t *buf)
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

int check_button_sequence(void)
{
	char buf[24];
	char sn[24];

	sn[0] = '\0';

	display_set_text_color();

	display_print_string(" *");

	if (trigger_wait_until(TRIG_OFF, 3000))
		goto out_fail;

	/* Press trigger 5 times and wait one second */
	printf("Sequence 1 start\n");
	if (seq(5) == FAIL)
		goto out_fail;
	if (trigger_wait_until(TRIG_ON, 1000) == OK)
		goto out_fail;
	printf("Sequence 1 ok\n");

	display_print_string("**");

	/* Press trigger 6 times and wait two seconds */
	printf("Sequence 2 start\n");
	if (seq(6) == FAIL)
		goto out_fail;
	if (trigger_wait_until(TRIG_ON, 2000) == OK)
		goto out_fail;
	printf("sequence 2 ok\n");

	get_serial_number((uint8_t *)sn);
	/* If serial is empty, we print a star */
	if (sn[0] == 0xff || strlen(sn) == 0) {
		sn[0] = '*';
		sn[1] = '\0';
	}

	snprintf(buf, 19, "Recovery %s", sn);
	display_print_string(buf);

	return OK;

out_fail:
	/* Clear display */
	display_print_string("                      ");
	return FAIL;
}
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
int check_recovery_sequence(void)
{
	printf("Checking cable\n");
	if (!get_usb_cable_state())
		return FAIL;

	/* Press trigger 5 times and wait one second */
	printf("Sequence 1 start\n");
	if (seq(5) == FAIL)
		return FAIL;
	if (trigger_wait_until(TRIG_ON, 1000) == OK)
		return FAIL;
	printf("Sequence 1 ok\n");

	/* Press trigger 6 times and wait two seconds */
	printf("Sequence 2 start\n");
	leds_battery_solid(20);
	if (seq(6) == FAIL)
		return FAIL;
	if (trigger_wait_until(TRIG_ON, 2000) == OK)
		return FAIL;
	printf("Sequence 2 ok\n");

	leds_boot();

	return OK;
}

int check_button_sequence(void)
{
	u16 soc;

	env_set("factorydefault_bootarg", "");

	printf("Check button sequence\n");

	/* Hold button 4 seconds in total to see battery SOC */
	if (trigger_wait_until(TRIG_OFF, SEQ_BATTERY_STATUS_START_MS) == OK)
		return FAIL;
	fuelgauge_get_state_of_charge(&soc);
	leds_battery_pulse(soc);
	printf("Displaying battery charge\n");

	/* Turn off the leds and the system after SOC if button is released */
	mdelay(SEQ_DISPLAY_SOC_MS);
	if (get_button_state() == TRIG_OFF)
		goto out_fail;

	printf("LED sequence 1\n");
	leds_all_blink_slow();
	/* If trigger is released we enter recovery sequence */
	if (trigger_wait_until(TRIG_OFF, SEQ_SLOW_FDEFAULT_MS) == OK) {
		if (check_recovery_sequence() == FAIL)
			goto out_fail;

		return OK;
	}

	printf("LED sequence 2\n");
	leds_all_blink_fast();
	if (trigger_wait_until(TRIG_OFF, SEQ_FAST_FDEFAULT_MS) == OK)
		goto out_fail;

	printf("LED sequence 3\n");
	leds_battery_solid(100);
	/* If trigger is released while leds are on we do a factory reset */
	if (trigger_wait_until(TRIG_OFF, SEQ_END_MS) == FAIL)
		goto out_fail;

	printf("Factory default\n");
	env_set("factorydefault_bootarg", "factorydefault");
	leds_boot();

	return FAIL;

out_fail:
	leds_off();
	power_off();

	return FAIL;
}
#endif

static void init_trigger_button(void)
{
	int ret;

	mx7ulp_iomux_setup_multiple_pads(btn_trig_pad, ARRAY_SIZE(btn_trig_pad));

	ret = dm_gpio_lookup_name(TRIGGER_PIN, &trigger_gpio_desc);
	if (ret) {
		printf("%s lookup %s failed ret = %d\n", __func__, TRIGGER_PIN, ret);
		return;
	}

	ret = dm_gpio_request(&trigger_gpio_desc, "trigger_btn");
	if (ret) {
		printf("%s request trigger_btn failed ret = %d\n", __func__, ret);
		return;
	}

	ret = dm_gpio_set_dir_flags(&trigger_gpio_desc, GPIOD_IS_IN);
	if (ret) {
		printf("%s set dir flags failed ret = %d\n", __func__, ret);
		return;
	}
}

int do_recoverytrigger(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int res = FAIL;

	printf("Check recovery\n");

	pcc_clock_enable(GPIO_PER_CLK, true);
	init_trigger_button();

	if (get_button_state() == TRIG_OFF)
		return FAIL;

	// Check recovery sequence
	printf("Trigger on, check recovery.\n");
	res = check_button_sequence();

	return res;
}

U_BOOT_CMD(recoverytrigger, 2, 1, do_recoverytrigger,
	   "do_recoverytrigger",
	   "do_recoverytrigger"
);
