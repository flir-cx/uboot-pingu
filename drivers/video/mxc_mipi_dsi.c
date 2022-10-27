
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

#include <common.h>
#include <mipi_dsi.h>
#include "mipi_common.h"
#include <linux/fb.h>
#include <asm/arch/crm_regs.h>
#include "mxc_mipi_dsi.h"
#include <linux/delay.h>
#include <asm/io.h>
#include <ipu_pixfmt.h>

#define DISPDRV_MIPI			"mipi_dsi"
#define ROUND_UP(x)			((x)+1)
#define NS2PS_RATIO			(1000)
#define NUMBER_OF_CHUNKS		(0x8)
#define NULL_PKT_SIZE			(0x8)
#define PHY_BTA_MAXTIME			(0xd00)
#define PHY_LP2HS_MAXTIME		(0x40)
#define PHY_HS2LP_MAXTIME		(0x40)
#define	PHY_STOP_WAIT_TIME		(0x20)
#define	DSI_CLKMGR_CFG_CLK_DIV		(0x107)
#define DSI_GEN_PLD_DATA_BUF_ENTRY	(0x10)
#define	MIPI_MUX_CTRL(v)		(((v) & 0x3) << 4)
#define	MIPI_LCD_SLEEP_MODE_DELAY	(120)
#define	MIPI_DSI_REG_RW_TIMEOUT		(20)
#define	MIPI_DSI_PHY_TIMEOUT		(10)


#define BITS_PER_BYTE				8


struct _mipi_dsi_phy_pll_clk {
	u32		max_phy_clk;
	u32		config;
};


/* configure data for DPHY PLL 27M reference clk out */
static const struct _mipi_dsi_phy_pll_clk mipi_dsi_phy_pll_clk_table[] = {
	{1000, 0x74}, /*  950-1000MHz	*/
	{950,  0x54}, /*  900-950Mhz	*/
	{900,  0x34}, /*  850-900Mhz	*/
	{850,  0x14}, /*  800-850MHz	*/
	{800,  0x32}, /*  750-800MHz	*/
	{750,  0x12}, /*  700-750Mhz	*/
	{700,  0x30}, /*  650-700Mhz	*/
	{650,  0x10}, /*  600-650MHz	*/
	{600,  0x2e}, /*  550-600MHz	*/
	{550,  0x0e}, /*  500-550Mhz	*/
	{500,  0x2c}, /*  450-500Mhz	*/
	{450,  0x0c}, /*  400-450MHz	*/
	{400,  0x4a}, /*  360-400MHz	*/
	{360,  0x2a}, /*  330-360Mhz	*/
	{330,  0x48}, /*  300-330Mhz	*/
	{300,  0x28}, /*  270-300MHz	*/
	{270,  0x08}, /*  250-270MHz	*/
	{250,  0x46}, /*  240-250Mhz	*/
	{240,  0x26}, /*  210-240Mhz	*/
	{210,  0x06}, /*  200-210MHz	*/
	{200,  0x44}, /*  180-200MHz	*/
	{180,  0x24}, /*  160-180MHz	*/
	{160,  0x04}, /*  150-160MHz	*/
};


static inline void mipi_dsi_write_register(struct mipi_dsi_info *mipi_dsi, u32 reg, u32 val)
{
	writel(val, MIPI_DSI_IPS_BASE_ADDR + reg);
}

static inline void mipi_dsi_read_register(struct mipi_dsi_info *mipi_dsi, u32 reg, u32 *val)
{
	*val = readl(MIPI_DSI_IPS_BASE_ADDR + reg);
}

static void mipi_dsi_dphy_init(struct mipi_dsi_info *mipi_dsi, u32 cmd, u32 data)
{
	u32 val;
	u32 timeout = 0;

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL,
			DSI_PHY_IF_CTRL_RESET);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_POWERUP);

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL1,
		(0x10000 | cmd));
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 2);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL1, (0 | data));
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 2);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	val = DSI_PHY_RSTZ_EN_CLK | DSI_PHY_RSTZ_DISABLE_RST |
			DSI_PHY_RSTZ_DISABLE_SHUTDOWN;
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_RSTZ, val);

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	while ((val & DSI_PHY_STATUS_LOCK) != DSI_PHY_STATUS_LOCK) {
        mdelay(1);
		timeout++;
		if (timeout == MIPI_DSI_PHY_TIMEOUT) {
			printf("Error: phy lock timeout!\n");
			break;
		}
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	}
	timeout = 0;
	while ((val & DSI_PHY_STATUS_STOPSTATE_CLK_LANE) !=
			DSI_PHY_STATUS_STOPSTATE_CLK_LANE) {
        mdelay(1);
		timeout++;
		if (timeout == MIPI_DSI_PHY_TIMEOUT) {
			printf("Error: phy lock lane timeout!\n");
			break;
		}
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	}
}


int mipi_dsi_pkt_write(struct mipi_dsi_info *mipi_dsi,u8 data_type, const u32 *buf, int len)
{
	u32 val;
	u32 status = 0;
	int write_len = len;
	uint32_t timeout = 0;

	if (len) {
		/* generic long write command */
		while (len / DSI_GEN_PLD_DATA_BUF_SIZE) {
			mipi_dsi_write_register(mipi_dsi,MIPI_DSI_GEN_PLD_DATA, *buf);
			buf++;
			len -= DSI_GEN_PLD_DATA_BUF_SIZE;
			mipi_dsi_read_register(mipi_dsi,MIPI_DSI_CMD_PKT_STATUS,
								&status);
			while ((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) ==
					 DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) {
				udelay(50);
				timeout++;
				if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
					return -1;
				mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
								&status);
			}
		}
		/* write the remainder bytes */
		if (len > 0) {
			while ((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) ==
					 DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) {
				udelay(50);
				timeout++;
				if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
					return -1;
				mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
								&status);
			}
			mipi_dsi_write_register(mipi_dsi,MIPI_DSI_GEN_PLD_DATA, *buf);
		}

		val = data_type | ((write_len & DSI_GEN_HDR_DATA_MASK)
			<< DSI_GEN_HDR_DATA_SHIFT);
	} else {
		/* generic short write command */
		val = data_type | ((*buf & DSI_GEN_HDR_DATA_MASK)
			<< DSI_GEN_HDR_DATA_SHIFT);
	}

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &status);
	while ((status & DSI_CMD_PKT_STATUS_GEN_CMD_FULL) ==
			 DSI_CMD_PKT_STATUS_GEN_CMD_FULL) {
		udelay(50);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -1;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
				&status);
	}
	mipi_dsi_write_register(mipi_dsi,MIPI_DSI_GEN_HDR, val);

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &status);
	while (!((status & DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY) ==
			 DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY) ||
			!((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY) ==
			DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY)) {
		udelay(50);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -1;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
				&status);
	}
	return 0;
}


static void mipi_dsi_enable_controller(struct mipi_dsi_info *mipi_dsi)
{
	u32		val;
	u32		lane_byte_clk_period;
	struct  fb_videomode *mode = mipi_dsi->mode;
	struct  mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
							DSI_PWRUP_RESET);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_RSTZ,
							DSI_PHY_RSTZ_RST);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CLKMGR_CFG,
							DSI_CLKMGR_CFG_CLK_DIV);

	if (!(mode->sync & FB_SYNC_VERT_HIGH_ACT))
		val = DSI_DPI_CFG_VSYNC_ACT_LOW;
	if (!(mode->sync & FB_SYNC_HOR_HIGH_ACT))
		val |= DSI_DPI_CFG_HSYNC_ACT_LOW;
	if ((mode->sync & FB_SYNC_OE_LOW_ACT))
		val |= DSI_DPI_CFG_DATAEN_ACT_LOW;
	if (MIPI_RGB666_LOOSELY == lcd_config->dpi_fmt)
		val |= DSI_DPI_CFG_EN18LOOSELY;
	val |= (lcd_config->dpi_fmt & DSI_DPI_CFG_COLORCODE_MASK)
			<< DSI_DPI_CFG_COLORCODE_SHIFT;
	val |= (lcd_config->virtual_ch & DSI_DPI_CFG_VID_MASK)
			<< DSI_DPI_CFG_VID_SHIFT;
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_DPI_CFG, val);

	val = DSI_PCKHDL_CFG_EN_BTA |
			DSI_PCKHDL_CFG_EN_ECC_RX |
			DSI_PCKHDL_CFG_EN_EOTP_RX |
			DSI_PCKHDL_CFG_EN_EOTP_TX |
			DSI_PCKHDL_CFG_EN_CRC_RX;

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PCKHDL_CFG, val);

	val = (mode->xres & DSI_VID_PKT_CFG_VID_PKT_SZ_MASK)
			<< DSI_VID_PKT_CFG_VID_PKT_SZ_SHIFT;
	val |= (NUMBER_OF_CHUNKS & DSI_VID_PKT_CFG_NUM_CHUNKS_MASK)
			<< DSI_VID_PKT_CFG_NUM_CHUNKS_SHIFT;
	val |= (NULL_PKT_SIZE & DSI_VID_PKT_CFG_NULL_PKT_SZ_MASK)
			<< DSI_VID_PKT_CFG_NULL_PKT_SZ_SHIFT;
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_PKT_CFG, val);

	/* enable LP mode when TX DCS cmd and enable DSI command mode */
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG,
							MIPI_DSI_CMD_MODE_CFG_EN_LOWPOWER);

	/* mipi lane byte clk period in ns unit */
	lane_byte_clk_period = NS2PS_RATIO /
			(lcd_config->max_phy_clk / BITS_PER_BYTE);
	val  = ROUND_UP(mode->hsync_len * mode->pixclock /
					NS2PS_RATIO / lane_byte_clk_period)
			<< DSI_TME_LINE_CFG_HSA_TIME_SHIFT;
	val |= ROUND_UP(mode->left_margin * mode->pixclock /
					NS2PS_RATIO / lane_byte_clk_period)
			<< DSI_TME_LINE_CFG_HBP_TIME_SHIFT;
	val |= ROUND_UP((mode->left_margin + mode->right_margin +
					 mode->hsync_len + mode->xres) * mode->pixclock
					/ NS2PS_RATIO / lane_byte_clk_period)
			<< DSI_TME_LINE_CFG_HLINE_TIME_SHIFT;
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_TMR_LINE_CFG, val);

	val = ((mode->vsync_len & DSI_VTIMING_CFG_VSA_LINES_MASK)
		   << DSI_VTIMING_CFG_VSA_LINES_SHIFT);
	val |= ((mode->upper_margin & DSI_VTIMING_CFG_VBP_LINES_MASK)
			<< DSI_VTIMING_CFG_VBP_LINES_SHIFT);
	val |= ((mode->lower_margin & DSI_VTIMING_CFG_VFP_LINES_MASK)
			<< DSI_VTIMING_CFG_VFP_LINES_SHIFT);
	val |= ((mode->yres & DSI_VTIMING_CFG_V_ACT_LINES_MASK)
			<< DSI_VTIMING_CFG_V_ACT_LINES_SHIFT);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VTIMING_CFG, val);

	val = ((PHY_BTA_MAXTIME & DSI_PHY_TMR_CFG_BTA_TIME_MASK)
		   << DSI_PHY_TMR_CFG_BTA_TIME_SHIFT);
	val |= ((PHY_LP2HS_MAXTIME & DSI_PHY_TMR_CFG_LP2HS_TIME_MASK)
			<< DSI_PHY_TMR_CFG_LP2HS_TIME_SHIFT);
	val |= ((PHY_HS2LP_MAXTIME & DSI_PHY_TMR_CFG_HS2LP_TIME_MASK)
			<< DSI_PHY_TMR_CFG_HS2LP_TIME_SHIFT);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TMR_CFG, val);

	val = (((lcd_config->data_lane_num - 1) &
			DSI_PHY_IF_CFG_N_LANES_MASK)
		   << DSI_PHY_IF_CFG_N_LANES_SHIFT);
	val |= ((PHY_STOP_WAIT_TIME & DSI_PHY_IF_CFG_WAIT_TIME_MASK)
			<< DSI_PHY_IF_CFG_WAIT_TIME_SHIFT);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CFG, val);

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST0, &val);
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST1, &val);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_ERROR_MSK0, 0);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_ERROR_MSK1, 0);

	//mipi_dsi_dphy_init(mipi_dsi, DSI_PHY_CLK_INIT_COMMAND,
	//				   mipi_dsi->dphy_pll_config, early);

}

static inline void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
	bool cmd_mode)
{
	u32	val;

	if (cmd_mode) {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_RESET);
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val |= MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, 0);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_POWERUP);
	} else {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_RESET);
		 /* Disable Command mode when tranfering video data */
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val &= ~MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		val = DSI_VID_MODE_CFG_EN | /*DSI_VID_MODE_CFG_EN_BURSTMODE |*/
				DSI_VID_MODE_CFG_EN_LP_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_POWERUP);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL,
				DSI_PHY_IF_CTRL_TX_REQ_CLK_HS);
	}
}

void mipi_clk_enable(void)
{
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int reg= readl(&mxc_ccm->CCGR3);
	reg |= MXC_CCM_CCGR3_MIPI_CORE_CFG_MASK;
	writel(reg, &mxc_ccm->CCGR3);
}

int mxc_mipi_dsi_enable(void)
{
	struct mipi_dsi_info mipi_dsi;
	int i;

	mipid_otm1287a_get_lcd_videomode(&mipi_dsi.mode, &mipi_dsi.lcd_config);
#if 0  // Handled through displays in mx6ec101.c and board_video_skip() in video.c
    ipuv3_fb_init(mipi_dsi.mode, 0,
               IPU_PIX_FMT_RGB24);
#endif
	mipi_clk_enable();

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_phy_pll_clk_table); i++) {
		if (mipi_dsi_phy_pll_clk_table[i].max_phy_clk <
				mipi_dsi.lcd_config->max_phy_clk)
			break;
	}
	if ((i == ARRAY_SIZE(mipi_dsi_phy_pll_clk_table)) ||
		(mipi_dsi.lcd_config->max_phy_clk >
			mipi_dsi_phy_pll_clk_table[0].max_phy_clk)) {
		printf("failed to find data in"
				"mipi_dsi_phy_pll_clk_table.\n");
		return -1;
	}
	mipi_dsi.dphy_pll_config = mipi_dsi_phy_pll_clk_table[--i].config;
	debug("dphy_pll_config:0x%x.\n", mipi_dsi.dphy_pll_config);

	mipi_dsi_enable_controller(&mipi_dsi);

	mipi_dsi_dphy_init(&mipi_dsi, DSI_PHY_CLK_INIT_COMMAND,
					   mipi_dsi.dphy_pll_config);

	mipid_otm1287a_lcd_setup(&mipi_dsi);

	mdelay(1);

	mipi_dsi_set_mode(&mipi_dsi, false);

	return 0;
}

