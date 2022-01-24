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

#define GPIO_SPI1_SCLK     IMX_GPIO_NR(5, 22)
#define GPIO_SPI1_MOSI     IMX_GPIO_NR(5, 23)
#define GPIO_SPI1_MISO     IMX_GPIO_NR(5, 24)
#define GPIO_SPI1_CS       IMX_GPIO_NR(5, 28)

#ifdef CONFIG_FPGA_XILINX
#define GPIO_FPGA_PROGRAM_n         IMX_GPIO_NR(5, 25)
#define GPIO_FPGA_INIT_n            IMX_GPIO_NR(5, 26)
#define GPIO_FPGA_CONF_DONE         IMX_GPIO_NR(5, 27)
#endif

#ifdef  CONFIG_FPGA_ALTERA
#define GPIO_FPGA_CONFIG_n          IMX_GPIO_NR(5, 25)
#define GPIO_FPGA_STATUS_n          IMX_GPIO_NR(5, 26)
#define GPIO_FPGA_CONF_DONE         IMX_GPIO_NR(5, 27)
#define GPIO_FPGA_CE                IMX_GPIO_NR(4, 10)
#endif
void setup_spinor(void);
#endif
