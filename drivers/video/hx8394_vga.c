// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 * Author(s): Rúni Eriksen <runi.eriksen@teledyne.com> for FLIR Systems.
 */
#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>

/*** Manufacturer Command Set ***/
#define MCS_SCMD_NOP 0x00 /* No Operation */
#define MCS_SCMD_SWRESET 0x01 /* Software Reset */
#define MCS_SCMD_RDDIDIF 0x04 /* Read Display Identification Information */
#define MCS_SCMD_RDNUMPE 0x05 /* Read Number of DSI Parity Error */
#define MCS_SCMD_RDRED 0x06 /* Read Red Colour */
#define MCS_SCMD_RDGREEN 0x07 /* Read Green Colour */
#define MCS_SCMD_RDBLUE 0x08 /* Read Blue Colour */
#define MCS_SCMD_RDDST 0x09 /* Read Display Status */
#define MCS_SCMD_RDDPM 0x0A /* Read Display Power Mode */
#define MCS_SCMD_RDDMACTL 0x0B /* Read Display MADCTL */
#define MCS_SCMD_RDDCOLMOD 0x0C /* Read Display Pixel Format */
#define MCS_SCMD_RDDIM 0x0D /* Read Display Image Mode */
#define MCS_SCMD_RDDSM 0x0E /* Read Display Signal Mode */
#define MCS_SCMD_RDDSDR 0x0F /* Read Display Self-Diagnostic Result */
#define MCS_SCMD_SLPIN 0x10 /* Sleep In */
#define MCS_SCMD_SLPOUT 0x11 /* Sleep Out */
#define MCS_SCMD_NORON 0x13 /* Normal Display Mode On */
#define MCS_SCMD_INVOFF 0x20 /* Display Inversion Off */
#define MCS_SCMD_INVON 0x21 /* Display Inversion On */
#define MCS_SCMD_ALLPOFF 0x22 /* All Pixel Off (Black) */
#define MCS_SCMD_ALLPOON 0x23 /* All Pixel On (White) */
#define MCS_SCMD_GAMSET 0x26 /* Gamma Set */
#define MCS_SCMD_DISPOFF 0x28 /* Display Off */
#define MCS_SCMD_DISPON 0x29 /* Display On */
#define MCS_SCMD_RAMWR 0x2C /* Memory Write */
#define MCS_SCMD_TEOFF 0x34 /* Tearing Effect Line Off */
#define MCS_SCMD_TEON 0x35 /* Tearing Effect Line On */
#define MCS_SCMD_MADCTL 0x36 /* Memory Access Control */
#define MCS_SCMD_IDMOFF 0x38 /* Idle Mode Off */
#define MCS_SCMD_IDMON 0x39 /* Idle Mode On */
#define MCS_SCMD_COLMOD 0x3A /* 0 */
#define MCS_SCMD_RAMWRCON 0x3C /* Memory Write */
#define MCS_SCMD_TESL 0x44 /* TESL */
#define MCS_SCMD_GETSCAN 0x45 /* Return the Current Scanline */
#define MCS_SCMD_WRDISBV 0x51 /* Write Display Brightness */
#define MCS_SCMD_RDDISBV 0x52 /* Read Display Brightness */
#define MCS_SCMD_WRCTRLD 0x53 /* Write CTRL Display */
#define MCS_SCMD_RDCTRLD 0x54 /* Read CTRL Display */
#define MCS_SCMD_WRCABC 0x55 /* Write Adaptive Brightness Control */
#define MCS_SCMD_RDCABC 0x56 /* Read Adaptive Brightness Control */
#define MCS_SCMD_WRCABCMB 0x5E /* Write CABC Minimum Brightness */
#define MCS_SCMD_RDCABCMB 0x5F /* Read CABC Minimum Brightness */
#define MCS_SCMD_RDABCSDR 0x68 /* Read Automatic Brightness Control Self-Diagnostic Result */
#define MCS_SCMD_WRIMCOL 0x80 /* Write Idle Mode Color */
#define MCS_SCMD_RDIMCOL 0x81 /* Read Idle Mode Color */
#define MCS_SCMD_READ_DDB_START 0xA1 /* Read the DDB from the Provided Location */
#define MCS_SCMD_READ_DDB_CONTINUE 0xA8 /* Continue Reading the DDB from the Last Read Location */
#define MCS_SCMD_RDID1 0xDA /* Read ID1 */
#define MCS_SCMD_RDID2 0xDB /* Read ID2 */
#define MCS_SCMD_RDID3 0xDC /* Read ID3 */

#define MCS_UCMD_SETAUTO 0xB0
#define MCS_UCMD_SETPOWER 0xB1
#define MCS_UCMD_SETDISP 0xB2
#define MCS_UCMD_SETCYC 0xB4
#define MCS_UCMD_SETVCOM 0xB6
#define MCS_UCMD_SETTE 0xB7
#define MCS_UCMD_SETSENSOR 0xB8
#define MCS_UCMD_SETEXTC 0xB9
#define MCS_UCMD_SETMIPI 0xBA
#define MCS_UCMD_SETOPT 0xBB
#define MCS_UCMD_SET_BANK 0xBD
#define MCS_UCMD_SETSTBA 0xC0
#define MCS_UCMD_SETDGCLUT 0xC1
#define MCS_UCMD_SETID 0xC3
#define MCS_UCMD_SETDDB 0xC4
#define MCS_UCMD_SETCABC 0xC9
#define MCS_UCMD_SETCABCGAIN 0xCA
#define MCS_UCMD_SETPANEL 0xCC
#define MCS_UCMD_SETOFFSET 0xD2
#define MCS_UCMD_SETGIP_0 0xD3
#define MCS_UCMD_SETIOOPT 0xD4
#define MCS_UCMD_SETGIP_1 0xD5
#define MCS_UCMD_SETGIP_2 0xD6
#define MCS_UCMD_SETGPO 0xD9
#define MCS_UCMD_SETSCALING 0xDD
#define MCS_UCMD_SET1BPP 0xDF
#define MCS_UCMD_SETGAMMA 0xE0
#define MCS_UCMD_SETCHEMODE 0xE4
#define MCS_UCMD_SETCHE 0xE5
#define MCS_UCMD_SETCHESEL 0xE6
#define MCS_UCMD_SET_SP_CMD 0xE9
#define MCS_UCMD_SETREADINDEX 0xFE

struct hx8394_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;
	struct gpio_desc reset;
};

static const struct display_timing default_timing = {
	.pixelclock.typ		= 46894000,
	.vactive.typ		= 480,
	.hactive.typ		= 640,
	.vfront_porch.typ	= 0,
	.vback_porch.typ	= 0,
	.vsync_len.typ		= 20,
	.hfront_porch.typ	= 26,
	.hback_porch.typ	= 15,
	.hsync_len.typ		= 2,
};

struct reg_value {
	u8	command; // mipi command
	u8	delay; //delay in ms
	u8	buf_size; //buffer size for long commands, for short set it to 0
	u8	buf[60];
};

static struct reg_value lcd_setup[] = {
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 4,
		.buf = {MCS_UCMD_SETEXTC, 0xFF, 0x83, 0x94}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 11,
		.buf = {MCS_UCMD_SETPOWER, 0x48, 0x14, 0x74,
			0x09, 0x33, 0x54, 0x71, 0x31, 0x4D, 0x2F}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 7,
		.buf = {MCS_UCMD_SETMIPI, 0x61, 0x03, 0x68,
			0x6B, 0xB2, 0xC0}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 22,
		.buf = {MCS_UCMD_SETCYC, 0x01, 0x70, 0x01,
			0x70, 0x01, 0x70, 0x01, 0x0C, 0x76, 0x35,
			0x00, 0x3F, 0x01, 0x70, 0x01, 0x70, 0x01,
			0x70, 0x01, 0x0C, 0x76}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 34,
		.buf = {MCS_UCMD_SETGIP_0, 0x00, 0x00, 0x00,
			0x00, 0x08, 0x08, 0x00, 0x00, 0x32, 0x10,
			0x03, 0x00, 0x03, 0x32, 0x11, 0xE9, 0x01,
			0xE9, 0x32, 0x10, 0x08, 0x00, 0x00, 0x37,
			0x03, 0x05, 0x05, 0x37, 0x05, 0x05, 0x17,
			0x06, 0x40}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 7,
		.buf = {MCS_UCMD_SETDISP, 0x00, 0x90, 0x00,
			0x0C, 0x0D, 0xFF}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 8,
		.buf = {0xBF, 0x40, 0x81, 0x50, 0x00, 0x1A,
			0xFC, 0x01}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 45,
		.buf = {MCS_UCMD_SETGIP_1, 0x18, 0x18, 0x24,
			0x25, 0x19, 0x19, 0x18, 0x18, 0x18, 0x18,
			0x06, 0x07, 0x04, 0x05, 0x02, 0x03, 0x00,
			0x01, 0x20, 0x21, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 45,
		.buf = {MCS_UCMD_SETGIP_2, 0x19, 0x19, 0x21,
			0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			0x01, 0x00, 0x03, 0x02, 0x05, 0x04, 0x07,
			0x06, 0x25, 0x24, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			0x18, 0x18, 0x18, 0x18, 0x18, 0x18}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 59,
		.buf = {MCS_UCMD_SETGAMMA, 0x00, 0x13, 0x20,
			0x27, 0x2C, 0x2F, 0x30, 0x30, 0x5E, 0x6D,
			0x7C, 0x78, 0x7F, 0x8E, 0x92, 0x94, 0xA1,
			0xA4, 0xA0, 0xAE, 0xBB, 0x5C, 0x59, 0x5D,
			0x60, 0x63, 0x67, 0x73, 0x7F, 0x00, 0x13,
			0x20, 0x27, 0x2C, 0x2F, 0x30, 0x30, 0x5E,
			0x6D, 0x7C, 0x78, 0x7F, 0x8E, 0x92, 0x94,
			0xA1, 0xA4, 0xA0, 0xAE, 0xBB, 0x5C, 0x59,
			0x5D, 0x60, 0x63, 0x67, 0x73, 0x7F}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 2,
		.buf = {MCS_UCMD_SETPANEL, 0x0B}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 3,
		.buf = {MCS_UCMD_SETSTBA, 0x1F, 0x31}
	},
	{
		.command = MIPI_DSI_GENERIC_LONG_WRITE,
		.delay = 0,
		.buf_size = 3,
		.buf = {MCS_UCMD_SETVCOM, 0x20, 0x20}
	},
	{
		.command = MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		.delay = 0,
		.buf_size = 0,
		.buf = {MCS_UCMD_SETIOOPT, 0x02}
	},
	{
		.command = MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		.delay = 0,
		.buf_size = 0,
		.buf = {MCS_UCMD_SETOFFSET, 0x77}
	},
	{
		.command = MIPI_DSI_DCS_SHORT_WRITE_PARAM,
		.delay = 0,
		.buf_size = 0,
		.buf = {0xC6, 0xED}
	},
	{
		.command = MIPI_DSI_DCS_SHORT_WRITE,
		.delay = 150,
		.buf_size = 0,
		.buf = {MCS_SCMD_SLPOUT}
	},
	{
		.command = MIPI_DSI_DCS_SHORT_WRITE,
		.delay = 50,
		.buf_size = 0,
		.buf = {MCS_SCMD_DISPON}
	}
};

static void hx8394_init_sequence(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	int err;
	int i;
	debug("MIPI DSI LCD HX8363 setup.\n");

	for (i = 0; i < ARRAY_SIZE(lcd_setup); i++) {
		err = mipi_dsi_dcs_write_buffer(device, lcd_setup[i].buf, lcd_setup[i].buf_size);
		if (err < 0)
			dev_err(dev, "MIPI DSI DCS write buffer failed: %d\n", err);

		mdelay(lcd_setup[i].delay);
	}
}

static int hx8394_panel_enable_backlight(struct udevice *dev)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	int ret;

	ret = mipi_dsi_attach(device);
	if (ret < 0)
		return ret;

	hx8394_init_sequence(dev);

	ret = mipi_dsi_dcs_exit_sleep_mode(device);
	if (ret)
		return ret;

	mdelay(125);

	ret = mipi_dsi_dcs_set_display_on(device);
	if (ret)
		return ret;

	mdelay(20);

	if (priv->backlight) {
		ret = backlight_enable(priv->backlight);
		if (ret)
			return ret;
	}
	return 0;
}

static int hx8394_panel_get_display_timing(struct udevice *dev, struct display_timing *timings)
{
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *device = plat->device;

	memcpy(timings, &default_timing, sizeof(*timings));

	/* fill characteristics of DSI data link */
	if (device) {
		device->lanes = plat->lanes;
		device->format = plat->format;
		device->mode_flags = plat->mode_flags;
	}

	return 0;
}

static int hx8394_panel_of_to_plat(struct udevice *dev)
{
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret =  device_get_supply_regulator(dev, "power-supply",
						   &priv->reg);
		if (ret && ret != -ENOENT) {
			dev_err(dev, "Warning: cannot get power supply\n");
			return ret;
		}
	}

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "Warning: cannot get reset GPIO\n");
		if (ret != -ENOENT)
			return ret;
	}

	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "Cannot get backlight: ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int hx8394_panel_probe(struct udevice *dev)
{
	struct hx8394_panel_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
			return ret;
	}

	/* reset panel */
	dm_gpio_set_value(&priv->reset, true);
	mdelay(1);
	dm_gpio_set_value(&priv->reset, false);
	mdelay(10);

	/* fill characteristics of DSI data link */
	plat->lanes = 2;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			   MIPI_DSI_MODE_EOT_PACKET |
			   MIPI_DSI_MODE_VIDEO_HSE;

	return 0;
}

static int hx8394_panel_disable(struct udevice *dev)
{
	struct hx8394_panel_priv *priv = dev_get_priv(dev);

	dm_gpio_set_value(&priv->reset, true);

	return 0;
}

static const struct panel_ops hx8394_panel_ops = {
	.enable_backlight = hx8394_panel_enable_backlight,
	.get_display_timing = hx8394_panel_get_display_timing,
};

static const struct udevice_id hx8394_panel_ids[] = {
	{ .compatible = "himax,hx8394" },
	{ }
};

U_BOOT_DRIVER(hx8394_panel) = {
	.name			= "hx8394_panel",
	.id				= UCLASS_PANEL,
	.of_match		= hx8394_panel_ids,
	.ops			= &hx8394_panel_ops,
	.of_to_plat		= hx8394_panel_of_to_plat,
	.probe			= hx8394_panel_probe,
	.remove			= hx8394_panel_disable,
	.plat_auto	= sizeof(struct mipi_dsi_panel_plat),
	.priv_auto	= sizeof(struct hx8394_panel_priv),
};
