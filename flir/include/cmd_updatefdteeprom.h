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
#include "eeprom.h"

#ifndef __CMD_UPDATEFDTEEPROM_H
#define __CMD_UPDATEFDTEEPROM_H

void patch_fdt_eeprom(void *blob);

struct Eeprom_list{
    struct list_head list;
    struct Eeprom  eeprom;
};

#endif
