/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2022 FLIR Systems.
 *
 */
#ifndef MXCFB_OTM1287_H
#define MXCFB_OTM1287_H

#include "mxc_mipi_dsi.h"

void mipid_otm1287a_get_lcd_videomode(struct fb_videomode **mode,
				      struct mipi_lcd_config **data);

int mipid_otm1287a_lcd_setup(struct mipi_dsi_info *mipi_dsi);

#endif  // MXCFB_OTM1287_H
