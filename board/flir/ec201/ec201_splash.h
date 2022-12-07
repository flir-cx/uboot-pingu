#include <common.h>
#include <asm/io.h>
#include <splash.h>
#include <video.h>

#ifdef CONFIG_VIDEO_MXS
int splash_screen_prepare(void);
void splash_screen_update(void);
#endif
