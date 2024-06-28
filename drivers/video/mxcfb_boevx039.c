// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 FLIR Systems.
 *
 */

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <log.h>
#include <panel.h>
#include <asm/gpio.h>
#include <power/regulator.h>
#include <asm/mach-imx/video.h>

struct boevx039_panel_priv {
	struct udevice *reg;
	struct udevice *i2c_bus;
	u32 i2c_addr;
};

static int boe_detect(struct display_info_t const *dev)
{
	return 1;
}

struct display_info_t const displays[] = {{
		.bus	= 1,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_RGB24,
		.di = 0,
		.detect	= boe_detect,
		//.enable	= enable_boe_backlight,
		.mode	= {
			// 1024x768 @ 60 Hz , pixel clk @ 50,1MHz //
			.name = "BOE-XGA",
			.refresh = 60,
			.xres = 1024,
			.yres = 768,
			.pixclock = 19921,
			.left_margin = 8,
			.right_margin = 4,
			.upper_margin = 35,
			.lower_margin = 3,
			.hsync_len = 4,
			.vsync_len = 6,
			.sync = 0, // We don't have drdy FB_SYNC_OE_LOW_ACT,
			.vmode = FB_VMODE_NONINTERLACED,
			.flag = 0,
		} }
};

size_t display_count = ARRAY_SIZE(displays);

struct setup_entry_4b {
	// high part of reg addr
	u8 reg_h;
	u8 reg_l;
	u8 val_h;
	u8 val_l;
};

static struct setup_entry_4b vf_setup_4byte[] = {
	{0x26, 0x00, 0x00, 0x20}, // Not described!
	{0x53, 0x00, 0x00, 0x20}, // Not described!
	{0x51, 0x00, 0x00, 0xFF}, // WRDISBV DBV[7:0]
	{0x51, 0x01, 0x00, 0x03}, // WRDISBV DBV[9:8] 0x3FF Full brightness?
	{0x80, 0x00, 0x00, 0x00}, // RESCTRL1 D0 => 45 MHz
	{0x80, 0x01, 0x00, 0x00}, // RESCTRL1 NC[7:0] => x-axis resolution 0*4
	{0x80, 0x02, 0x00, 0xC0}, // RESCTRL1 NL[7:0] => y-axis resolution 192*4=768 (NL[8]=0 below)
	{0x80, 0x03, 0x00, 0x10}, // RESCTRL1 NC[8] = 1, NL[8] = 0, x-axis resolution 0x100*4=1024
	{0x80, 0x04, 0x00, 0x00}, // RESCTRL1 NC_DEC[7:0] => 0
	{0x80, 0x05, 0x00, 0x04}, // RESCTRL1 NC_DEC[10:8] => NC_DEC=0x0400 => 1024
	{0x81, 0x00, 0x00, 0x01}, // RESCTRL2 T1A[9:8] = 1
	{0x81, 0x01, 0x00, 0xD1}, // RESCTRL2 T1A[7:0] = 0xD1 => T1A=0x1D1 => 466 hsync dpclk
	{0x81, 0x02, 0x00, 0x00}, // RESCTRL2 VBPDA[9:8] = 0
	{0x81, 0x03, 0x00, 0x23}, // RESCTRL2 VBPDA[7:0] = 0x23 => 35
	{0x81, 0x04, 0x00, 0x00}, // RESCTRL2 VBFDA[9:8] = 0
	{0x81, 0x05, 0x00, 0x03}, // RESCTRL2 VBFDA[7:0] = 0x3 => 3
	{0x81, 0x06, 0x00, 0x01}, // RESCTRL2 PSELA[2:0] = 1, (0h=>1 VBP line, 7h=>10 VBP line) 1h?
	{0x82, 0x00, 0x00, 0x01}, // Not described!
	{0x82, 0x01, 0x00, 0xD1}, // Not described!
	{0x82, 0x02, 0x00, 0x00}, // Not described!
	{0x82, 0x03, 0x00, 0x23}, // Not described!
	{0x82, 0x04, 0x00, 0x00}, // Not described!
	{0x82, 0x05, 0x00, 0x03}, // Not described!
	{0x82, 0x06, 0x00, 0x03}, // Not described!
	{0x83, 0x00, 0x00, 0x80}, // Set RGB_DE_OPT=1 for RGB video mode 2, no data enable used
	{0x83, 0x01, 0x00, 0x0A}, // Set RGB_HBP, according to mail
	{0x35, 0x00, 0x00, 0x00}, // Not described!
	{0xFF, 0x00, 0x00, 0x5A}, // Not described!
	{0xFF, 0x01, 0x00, 0x81}, // Not described!
	{0xF9, 0x0D, 0x00, 0x40}, // Not described!
	{0xF9, 0x0E, 0x00, 0x47}, // Not described!
	{0xF9, 0x0F, 0x00, 0x4E}, // Not described!
	{0xF9, 0x10, 0x00, 0x55}, // Not described!
	{0xF9, 0x11, 0x00, 0x5C}, // Not described!
	{0xF9, 0x12, 0x00, 0x5E}, // Not described!
	{0xF9, 0x13, 0x00, 0x61}, // Not described!
	{0xF9, 0x14, 0x00, 0x64}, // Not described!
	{0xF9, 0x15, 0x00, 0x67}, // Not described!
	{0xF9, 0x16, 0x00, 0x6A}, // Not described!
	{0xF9, 0x17, 0x00, 0x6C}, // Not described!
	{0xF9, 0x18, 0x00, 0x6F}, // Not described!
	{0xF9, 0x19, 0x00, 0x72}, // Not described!
	{0xF9, 0x1A, 0x00, 0x75}, // Not described!
	{0xF9, 0x1B, 0x00, 0x78}, // Not described!
	{0xF9, 0x1C, 0x00, 0x7A}, // Not described!
	{0xF9, 0x1D, 0x00, 0x7D}, // Not described!
	{0xF9, 0x1E, 0x00, 0x80}, // Not described!
	{0xF9, 0x1F, 0x00, 0x83}, // Not described!
	{0xF9, 0x20, 0x00, 0x86}, // Not described!
	{0xF9, 0x21, 0x00, 0x88}, // Not described!
	{0xF9, 0x22, 0x00, 0x8B}, // Not described!
	{0xF9, 0x23, 0x00, 0x8E}, // Not described!
	{0xF9, 0x24, 0x00, 0x91}, // Not described!
	{0xF9, 0x25, 0x00, 0x94}, // Not described!
	{0xF9, 0x26, 0x00, 0x96}, // Not described!
	{0xF9, 0x27, 0x00, 0x99}, // Not described!
	{0xF9, 0x28, 0x00, 0x9C}, // Not described!
	{0xF9, 0x29, 0x00, 0x9F}, // Not described!
	{0xF9, 0x2A, 0x00, 0xA2}, // Not described!
	{0xF9, 0x2B, 0x00, 0xA4}, // Not described!
	{0xF9, 0x2C, 0x00, 0xA7}, // Not described!
	{0xF9, 0x2D, 0x00, 0xAA}, // Not described!
	{0xF9, 0x2E, 0x00, 0xAD}, // Not described!
	{0xF9, 0x2F, 0x00, 0xB0}, // Not described!
	{0xF4, 0x13, 0x00, 0x42}, // Not described!
	{0xF2, 0x07, 0x00, 0x11}  // Not described!
};

struct setup_entry_2b {
	// high part of reg addr
	u8 reg_h;
	u8 reg_l;
};

static struct setup_entry_2b vf_setup_2byte[] = {
	{0x11, 0x00}, // Sleep out
	{0x29, 0x00} // Display on
};

static int i2c_reg_write(struct boevx039_panel_priv *priv, u8 *data_out, int len)
{
	int ret;

	struct i2c_msg msg = {
		.addr	= priv->i2c_addr,
		.flags	= 0,
		.len	= len,
		.buf	= data_out,
	};
	struct dm_i2c_ops *ops = i2c_get_ops(priv->i2c_bus);

	if (!ops->xfer)
		return -EFAULT;

	ret = ops->xfer(priv->i2c_bus, &msg, 1);
	if (ret)
		log_err("%s: Error i2c_xfer 0x%02x%02x ret=%d\n",
			__func__, data_out[0], data_out[1], ret);

	// According to meeting with boe the controller needs 100us
	// to process the data
	udelay(100);

	if (len == 2)
		debug("%s: Write register 0x%02x%02x! %d\n",
		      __func__, data_out[0], data_out[1], ret);
	else if (len == 4)
		debug("%s: Write register 0x%02x%02x%02x%02x! %d\n",
		      __func__, data_out[0], data_out[1], data_out[2], data_out[3], ret);

	if (ret < 0)
		log_err("%s: Failed writing register 0x%02x%02x! %d\n",
			__func__, data_out[0], data_out[1], ret);

	return ret;
}

static int boe_disp_i2c_init(struct boevx039_panel_priv *priv)
{
	int i;
	int ret = 0;
	u8 data_out[4];

	for (i = 0; ret >= 0 && i < ARRAY_SIZE(vf_setup_4byte); ++i) {
		data_out[0] = vf_setup_4byte[i].reg_h;
		data_out[1] = vf_setup_4byte[i].reg_l;
		data_out[2] = vf_setup_4byte[i].val_h;
		data_out[3] = vf_setup_4byte[i].val_l;
		ret = i2c_reg_write(priv, data_out, 4);
	}

	for (i = 0; ret >= 0 && i < ARRAY_SIZE(vf_setup_2byte); ++i) {
		data_out[0] = vf_setup_2byte[i].reg_h;
		data_out[1] = vf_setup_2byte[i].reg_l;
		ret = i2c_reg_write(priv, data_out, 2);
	}

	return ret;
}

static int boevx039_panel_enable_backlight(struct udevice *dev)
{
	// Oled does not have a backlight device.
	return 0;
}

static int boevx039_panel_set_backlight(struct udevice *dev, int percent)
{
	// TODO: do we need this function or should it be removed?
	return 0;
}

static int boevx039_panel_of_to_plat(struct udevice *dev)
{
	int ret;
	u32 i2c_bus;
	u32 i2c_addr;
	struct boevx039_panel_priv *priv = dev_get_priv(dev);

	debug("%s:\n", __func__);

	if (IS_ENABLED(CONFIG_DM_REGULATOR)) {
		ret = uclass_get_device_by_phandle(UCLASS_REGULATOR, dev,
						   "power-supply", &priv->reg);
		if (ret) {
			debug("%s: Warning: cannot get power supply: ret=%d\n",
			      __func__, ret);
			if (ret != -ENOENT)
				return ret;
		}
	}

	ret = dev_read_u32(dev, "i2c_bus", &i2c_bus);
	if (ret) {
		debug("%s: No i2c_bus in device tree %d\n", __func__, i2c_bus);
		return ret;
	}
	ret = dev_read_u32(dev, "i2c_addr", &i2c_addr);
	if (ret) {
		debug("%s: No i2c_addr in device tree %d\n", __func__, i2c_addr);
		return ret;
	}
	priv->i2c_addr = i2c_addr;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &priv->i2c_bus);
	if (ret) {
		debug("%s: No bus %d\n", __func__, i2c_bus);
		return ret;
	}

	return 0;
}

static int boevx039_panel_probe(struct udevice *dev)
{
	struct boevx039_panel_priv *priv = dev_get_priv(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DM_REGULATOR) && priv->reg) {
		debug("%s: Enable regulator '%s'\n", __func__, priv->reg->name);
		ret = regulator_set_enable(priv->reg, true);
		if (ret)
			return ret;
	}

	ret = boe_disp_i2c_init(priv);
	if (ret) {
		debug("%s: Failed to init panel %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static const struct panel_ops boevx039_panel_ops = {
	.enable_backlight	= boevx039_panel_enable_backlight,
	.set_backlight		= boevx039_panel_set_backlight,
};

static const struct udevice_id boevx039_panel_ids[] = {
	{ .compatible = "boe,vx039x0m-nh0" },
	{ }
};

U_BOOT_DRIVER(boevx039_panel) = {
	.name	= "boevx039_panel",
	.id	= UCLASS_PANEL,
	.of_match = boevx039_panel_ids,
	.ops	= &boevx039_panel_ops,
	.of_to_plat	= boevx039_panel_of_to_plat,
	.probe		= boevx039_panel_probe,
	.priv_auto	= sizeof(struct boevx039_panel_priv),
};
