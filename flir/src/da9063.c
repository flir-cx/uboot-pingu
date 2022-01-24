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
#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <spi.h>
#include "../../../flir/include/da9063.h"
#include "../../../flir/include/da9063_regs.h"

extern struct spi_slave *slave;

static int pmic_access_page(unsigned char page)
{
    u8 din[2];
    u8 dout[2];
    dout[1] = page;
    dout[0] = (DA9063_REG_PAGE_CON <<1);

    if (spi_xfer(slave, 16, dout, din, SPI_XFER_BEGIN | SPI_XFER_END)) {
		printf("Cannot set PMIC page!\n");
		return -1;
	}
	return 0;
}

int pmic_read_reg(int reg, unsigned char *value)
{
	unsigned char page = reg / 0x80;
    u8 din[2];
    u8 dout[2];
    dout[1] = 0;
    dout[0] = (reg <<1) | DA9063_RW;

	if (pmic_access_page(page))
		return -1;

    if (spi_xfer(slave, 16, dout, din, SPI_XFER_BEGIN | SPI_XFER_END)) {
		return -1;
	}
    *value = din[1];

	/* return to page 0 by default */
	pmic_access_page(0);
	return 0;
}

int pmic_write_reg(int reg, unsigned char value)
{
	unsigned char page = reg / 0x80;
    u8 din[2];
    u8 dout[2];
    dout[1] = value;
    dout[0] = (reg <<1);

	if (pmic_access_page(page))
		return -1;

    if (spi_xfer(slave, 16, dout, din, SPI_XFER_BEGIN | SPI_XFER_END)) {
		return -1;
	}

	/* return to page 0 by default */
	pmic_access_page(0);
	return 0;
}

int pmic_write_bitfield(int reg, unsigned char mask,
			       unsigned char bfval)
{
	unsigned char value;

	if (pmic_read_reg(reg, &value) == 0) {
		value &= ~(mask);
		value |= (bfval);
		return pmic_write_reg(reg, value);
	}

	return -1;
}




static int do_pmic(struct cmd_tbl *cmdtp, int flag, int argc,
				char * const argv[])
{
	unsigned char value,cmd_write;
	int reg;

	if((argv[1][0] == 'r') && (argc == 3))
		cmd_write = 0;
	else if((argv[1][0] == 'w') && (argc == 4))
		cmd_write = 1;
	else
		return CMD_RET_USAGE;

	reg = simple_strtoul(argv[2], NULL, 16);

	if(cmd_write)
	{
		value = simple_strtoul(argv[3], NULL, 16);
		pmic_write_reg(reg,value);
	}
	else
	{
		pmic_read_reg(reg,&value);
	}

	printf("%x: %x \n",reg,value);

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
    pmic,	4,	0,	do_pmic,
    "read and write to DA9063 pmic registers",
    "pmic {r|w} {reg} {data}"
);

