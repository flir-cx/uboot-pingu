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
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/mx6-pins.h>

#ifndef __CMD_LOADFPGA_H
#define __CMD_LOADFPGA_H

/**
 * The reason to keep this for the while being is to have som idea
 * of if the spi_flash has been initialized from the board.
 *
 * A better solution for this is needed, DM_SPI_FLASH should probably
 * be used. probe/remove functions may be used for ownership of flash.
 *
 */
void setup_spinor(void);
#endif
