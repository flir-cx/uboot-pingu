
/*
 * Copyright (C) 2016 FLIR Systems.
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




#ifndef _DA9063_H
#define	_DA9063_H

int pmic_write_reg(int reg, unsigned char value);

int pmic_read_reg(int reg, unsigned char *value);

int pmic_write_bitfield(int reg, unsigned char mask,
			       unsigned char bfval);


#endif
