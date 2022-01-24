/*
 * Copyright (C) 2015 FLIR Systems.
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

#include "linux/list.h"
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/mx6-pins.h>


#ifndef __CMD_CHECKCHARGER_H
#define __CMD_CHECKCHARGER_H

#define PMU_REG_3P0  0x20C8120
#define USB_ANALOG_USB1_VBUS_DETECT_STAT 0x20C81C0
#define USB_ANALOG_USB1_VBUS_DETECT_STAT_VBUSVALID 1<<3
#define USB_ANALOG_USB1_CHRG_DETECT1 0x20C81b0
#define USB_ANALOG_USB1_CHRG_DETECT_STAT  0x20C81D0
#define BQ24298_I2C_ADDR 0x6b
#define USBPHY1_DEBUG 0x20C9050
#define USBPHY1_CTRL 0x20C9030




#endif
