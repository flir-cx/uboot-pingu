#include <common.h>
#include <command.h>
#include <errno.h>
#include <linux/delay.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>

#include "../../../flir/include/cmd_loadfpga.h"
#include "../../../flir/include/da9063.h"

static void power_up_fpga(void);
//static void power_up_fpga(void);

__weak int fpga_power(bool enable)
{
	printf("Weak fpga power, fpga_power should be defined in board\n");
	return 0;
};



#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
    #define LOG_MSG printf
#else
    #define LOG_MSG(...)
#endif


//FPGA Configuration is documented in 
// Altera: CV-52007 (2017.09.19)
//         Cyclone Device Handbook Volume 1, Chapter 7 Configuration
//
// EC501 FPGA: Altera 5CGXF7B6M157
// Configuration MSEL[4:0] = [10011] -- Active serial mode
//pin             CPU config                                     config fnctn
//SPI1_SCLK_FPGA  hiZ                                            disable_spi_bus
//SPI1_CS1_n      hiZ                                            disable_spi_bus
//SPI1_MOSI       hiZ                                            disable_spi_bus
//SPI1_MISO       hiZ                                            disable_spi_bus
//FPGA_CE_n       output, must be low during config,             power_up_fpga
//                set high to release spi
//CONFIG_n        output, config triggered by L->H transition    power_up_fpga 
//STATUS_n        input                                          power_up_fpga 
//CONF_DONE       input                                          power_up_fpga

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const ecspi1_pads2[] = {
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const no_ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__GPIO5_IO22  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__GPIO5_IO23  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__GPIO5_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void enable_spi_bus(bool enable)
{
	if (enable) {
		//restore cpu spi bus
		LOG_MSG("%s - Enable\n", __func__);
		imx_iomux_v3_setup_multiple_pads(ecspi1_pads2, ARRAY_SIZE(ecspi1_pads2));
	} else {
		//cpu spi bus conflicts with fpga spi bus, disable cpu bus
		LOG_MSG("%s - Disable\n", __func__);
		imx_iomux_v3_setup_multiple_pads(no_ecspi1_pads, ARRAY_SIZE(no_ecspi1_pads));
	}
}

static void power_up_fpga(void)
{
	LOG_MSG("%s\n", __func__);
	gpio_direction_output(GPIO_FPGA_CONFIG_n, 0);
	gpio_direction_input(GPIO_FPGA_CONF_DONE);
	gpio_direction_input(GPIO_FPGA_STATUS_n);
	enable_fpga(false);
	fpga_power(true);
	mdelay(10);
	enable_fpga(true);
}

void enable_fpga(bool enable)
{
	if (enable) {
		gpio_direction_output(GPIO_FPGA_CE, 0);
	} else {
		gpio_direction_output(GPIO_FPGA_CE, 1);
	}
	udelay(10);
}

static void start_fpga_configuration(void)
{
	LOG_MSG("%s\n", __func__);
	gpio_direction_output(GPIO_FPGA_CONFIG_n, 0);
	enable_fpga(true);
	mdelay(10);
	gpio_direction_output(GPIO_FPGA_CONFIG_n , 1);
}



static void request_fpga_config_pins(bool enable)
{
	if (enable) {
		gpio_request(GPIO_FPGA_CONFIG_n, "FPGA Config");
		gpio_request(GPIO_FPGA_STATUS_n, "FPGA Status");
		gpio_request(GPIO_FPGA_CE, "FPGA CE");
	} else {
		gpio_free(GPIO_FPGA_CONFIG_n);
		gpio_free(GPIO_FPGA_STATUS_n);
		gpio_free(GPIO_FPGA_CE);
	}
}


int do_load_fpga(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	LOG_MSG("%s\n", __func__);
	enable_spi_bus(false);
	request_fpga_config_pins(true);
	gpio_request(GPIO_FPGA_CONF_DONE, "FPGA CONF_DONE");
	power_up_fpga();

	printf("Loading FPGA\n");

	/* Just start config, don't wait for it to complete */
	start_fpga_configuration();
	mdelay(1000);
	request_fpga_config_pins(false);
	enable_spi_bus(true);

	if (gpio_get_value(GPIO_FPGA_CONF_DONE) == 1) {
		printf("FPGA Loading done\n");
		ret = CMD_RET_SUCCESS;
	} else {
		printf("FPGA Loading failed\n");
		ret = CMD_RET_FAILURE;
	}

	gpio_free(GPIO_FPGA_CONF_DONE);

	return ret;
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(loadFPGA, 2, 0, do_load_fpga,
	   "Start load of main FPGA. Add 't' to wait for config to complete.",
	   "[t]"
	   );
#endif

U_BOOT_CMD(flir_loadfpga, 2, 0, do_load_fpga,
	   "Start load of main FPGA. Add 't' to wait for config to complete.",
	   "[t]"
	   );
