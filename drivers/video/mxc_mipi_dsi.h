#ifndef __MIPI_DSI_H__
#define __MIPI_DSI_H__


#define FB_SYNC_OE_LOW_ACT   0x80000000


/* DPI interface pixel color coding map */
enum mipi_dsi_dpi_fmt {
	MIPI_RGB565_PACKED = 0,
	MIPI_RGB565_LOOSELY,
	MIPI_RGB565_CONFIG3,
	MIPI_RGB666_PACKED,
	MIPI_RGB666_LOOSELY,
	MIPI_RGB888,
};

struct mipi_lcd_config {
	u32				virtual_ch;
	u32				data_lane_num;
	/* device max DPHY clock in MHz unit */
	u32				max_phy_clk;
	enum mipi_dsi_dpi_fmt		dpi_fmt;
};



/* driver private data */
struct mipi_dsi_info {
	u32				dphy_pll_config;
	struct  fb_videomode		*mode;
	struct  mipi_lcd_config		*lcd_config;

};

int mxc_mipi_dsi_enable(void);
int mipi_dsi_pkt_write(struct mipi_dsi_info *mipi_dsi,u8 data_type, const u32 *buf, int len);

int mipid_otm1287a_lcd_setup(struct mipi_dsi_info *mipi_dsi);
void mipid_otm1287a_get_lcd_videomode(struct fb_videomode **mode,
		struct mipi_lcd_config **data);



#endif
