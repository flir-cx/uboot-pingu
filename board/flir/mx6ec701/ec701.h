/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 FLIR Systems.
 */
#ifndef __EC701_H
#define __EC701_H
#include <asm/arch-mx6/iomux.h>
#include <asm/arch-mx6/mx6-pins.h>
#include <asm/arch-mx6/mx6-ddr.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
			PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |	\
			PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
			PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |	\
			PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |				\
		       PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
		       PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#endif // __EC701_H
