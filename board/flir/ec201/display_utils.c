#include <dm.h>
#include <video.h>
#include <video_console.h>

void display_set_text_color(void)
{
	struct udevice *dev_console;
	struct video_priv *vid_priv;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	vid_priv = dev_get_uclass_priv(dev_console->parent);

	/* foreground color */
	vid_priv->fg_col_idx &= ~7;
	vid_priv->fg_col_idx |= 15;
	vid_priv->colour_fg = vid_console_color(
			vid_priv, vid_priv->fg_col_idx);

	/* background color, also mask the bold bit */
	vid_priv->bg_col_idx &= ~0xf;
	vid_priv->bg_col_idx |= 0;
	vid_priv->colour_bg = vid_console_color(
			vid_priv, vid_priv->bg_col_idx);
}

void display_print_string(char *s)
{
	char buf[64];
	struct udevice *dev_console;
	int xpos, ypos;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	xpos = (80 / 2) - (strlen(s) / 2);
	ypos = 19;

	vidconsole_position_cursor(dev_console, xpos, ypos);
	snprintf(buf, 64, "%s", s);
	vidconsole_put_string(dev_console, buf);

	video_sync(dev_console->parent, true);
}

void display_print_charge_level(int c)
{
	char buf[10];
	struct udevice *dev_console;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev_console)) {
		printf("no text console device found!\n");
		return;
	}

	vidconsole_position_cursor(dev_console, 40, 19);
	snprintf(buf, 10, "%d%%", c);
	vidconsole_put_string(dev_console, buf);
}

void display_draw_box(int x_start, int y_start, int width, int height, int color, void *framebuffer)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device(UCLASS_VIDEO, 0, &dev);
	if (ret) {
		printf("Couldnt get video device!\n");
		return;
	}

	int bpp = 4; /* Bytes per pixel */
	int stride = video_get_xsize(dev) * bpp; /* Bytes per row */
	int skip = stride - width * bpp; /* Bytes to jump */

	char *dst = framebuffer + (x_start * bpp) + (y_start * stride);

	while(height--) {
		for(int w = 0; w < width; w++) {
			*(u32*)dst = color;
			dst += bpp;
		}
		dst += skip;
	}

	video_sync(dev, true);
}
