/*
 * Copyright (C) 2015 FLIR Systems.
 * Copyright (C) 2017 FLIR Systems.
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

#ifndef __CMD_KBD_H
#define __CMD_KBD_H

#include <common.h>
#include <asm/gpio.h>
int get_gpio_ioexpander(unsigned nr);
int read_keys(char *buf);




#define KEYBOARD_IO_EXP_I2C_ADDR	0x20
#define KEYBOARD_BEIA_IO_EXP_I2C_ADDR   0x21
#define PCA9534_INPUT_PORT		0x0
#define RECOVERY_KEY                    "R"
#define SW_LOAD                         "S"

struct button_key {
	char const	*name;
	int		(*get_key) (unsigned);
	unsigned	gpnum;
	char		ident;
};

#define EVIOBUSMSK (KEYBOARD_IO_EXP_I2C_ADDR << 8)
#define BEIABUSMSK (KEYBOARD_BEIA_IO_EXP_I2C_ADDR << 8)

static struct button_key const buttons[] = {
  //	{"sw_load",	gpio_get_value, IMX_GPIO_NR(7, 11),	'S'},
	{"right",	get_gpio_ioexpander, EVIOBUSMSK | 0,	'R'},
	{"left",	get_gpio_ioexpander, EVIOBUSMSK | 1,	'L'},
	{"up",		get_gpio_ioexpander, EVIOBUSMSK | 2,	'U'},
	{"back",	get_gpio_ioexpander, EVIOBUSMSK | 4,	'B'},
	{"down",	get_gpio_ioexpander, EVIOBUSMSK | 5,	'D'},
	{"middle",	get_gpio_ioexpander, EVIOBUSMSK | 6,	'M'},
	{"sw_load",	get_gpio_ioexpander, BEIABUSMSK | 0,    'S'},
};


#endif
