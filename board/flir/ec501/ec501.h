#ifndef __EC501_H
#define __EC501_H
#include <asm/arch-mx6/iomux.h>
#include <asm/arch-mx6/mx6-pins.h>
#include <asm/arch-mx6/mx6-ddr.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
		      PAD_CTL_SPEED_MED |			\
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |			\
			PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
			PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |			\
			PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |	\
			PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |			\
			PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |	\
			PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |			\
			 PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
			 PAD_CTL_DSE_40ohm | PAD_CTL_HYS |		\
			 PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#ifdef CONFIG_FSL_ESDHC
/* The order of MMC controllers here must match that of CONFIG_MMCDEV_USDHCx
 * in the platform header
 */
struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC4_BASE_ADDR},
};
#endif

#endif
