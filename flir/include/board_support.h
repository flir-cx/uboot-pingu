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

#ifndef __BOARD_SUPPORT_H
#define __BOARD_SUPPORT_H
#include "../../flir/include/flir_generic.h"
#include "../../flir/include/eeprom.h"

int board_support_setup(struct Eeprom *ioboard, struct hw_support *hardware);

#endif
