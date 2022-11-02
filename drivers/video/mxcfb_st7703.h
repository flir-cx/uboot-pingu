/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2022 FLIR Systems.
 *
 */
#ifndef MXCFB_ST7703_H
#define MXCFB_ST7703_H

#include "mxc_mipi_dsi.h"

void mipid_st7703_get_lcd_videomode(struct fb_videomode **mode,
				    struct mipi_lcd_config **data);

int mipid_st7703_lcd_setup(struct mipi_dsi_info *mipi_dsi);

#endif  // MXCFB_ST7703_H
