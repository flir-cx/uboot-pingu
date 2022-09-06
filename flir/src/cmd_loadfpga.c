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


#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)


iomux_v3_cfg_t const ecspi1_pads[] = {
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

static void enable_spi_bus(void)
{
  LOG_MSG("%s\n",  __func__);  
	//restore cpu spi bus
         gpio_request(IMX_GPIO_NR(5, 28), "CS SPI1");
     //	 gpio_direction_output(IMX_GPIO_NR(5, 28),1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	gpio_direction_output(GPIO_SPI1_CS,1);
}

static void disable_spi_bus(void)
{
    LOG_MSG("%s\n",  __func__);
	//cpu spi bus conflicts with fpga spi bus, disable cpu bus
	imx_iomux_v3_setup_multiple_pads(no_ecspi1_pads,
					 ARRAY_SIZE(no_ecspi1_pads));
	gpio_request(GPIO_SPI1_SCLK, "test-spi0");
	gpio_direction_input(GPIO_SPI1_SCLK);
	gpio_request(GPIO_SPI1_MOSI, "test-spi1");
	gpio_direction_input(GPIO_SPI1_MOSI);
	gpio_request(GPIO_SPI1_MISO, "test-spi2");	
	gpio_direction_input(GPIO_SPI1_MISO);
	gpio_request(GPIO_SPI1_CS, "test-spi3");
	gpio_direction_output(GPIO_SPI1_CS,1);
	gpio_direction_input(GPIO_SPI1_CS);
}

#ifdef CONFIG_SYS_USE_SPINOR

#define HEADER_LENGTH  65536
static struct spi_flash *flash;

static int check_fpga_flash_header(void)
{
	char buf[6] = {0};
	int offset = 0;

	if (!flash)
	{
		printf("SPI FLASH probe failed\n");
		return 1; // try to load fpga anyway
	}

	offset = flash->size - HEADER_LENGTH;
	spi_flash_read(flash,offset,6,buf);

	if(memcmp(buf, "FLIR", 4))
	{
		printf("Error: FPGA header missing \n");
		return 0;
	}
	printf("FPGA header OK\n");

	//reset address to 0
	spi_flash_read(flash,0,6,buf);
	return 1;
}

void setup_spinor(void)
{
  LOG_MSG("%s\n",  __func__);
  	power_up_fpga();
	enable_spi_bus();
	udelay(10);
#if 0
	/* disabled for now, see BC-150 */
	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
				CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE) ;
#endif
}

#endif /* CONFIG_SYS_USE_SPINOR */

static void enable_fpga(void)
{
#if defined(CONFIG_FPGA_XILINX)

	/* n/a */

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CE, 0);
	udelay(10);

#endif
}

static void disable_fpga(void)
{
#if defined(CONFIG_FPGA_XILINX)

	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,0);
	gpio_direction_output(GPIO_FPGA_INIT_n ,0);

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CE, 1);
	udelay(10);
#endif
}

static void power_up_fpga(void)
{
        LOG_MSG("%s\n", __func__);
#if defined(CONFIG_FPGA_XILINX)

	gpio_request(GPIO_FPGA_PROGRAM_n, "test-gpio0");
	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,0);
	gpio_request(GPIO_FPGA_INIT_n, "test-gpio1");	
	gpio_direction_output(GPIO_FPGA_INIT_n ,0);
	gpio_request(GPIO_FPGA_CONF_DONE, "test-gpio2");		
	gpio_direction_input(GPIO_FPGA_CONF_DONE);
	fpga_power(true);
	mdelay(10);

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CONFIG_n, 0);
	gpio_direction_input(GPIO_FPGA_CONF_DONE);
	gpio_direction_input(GPIO_FPGA_STATUS_n);
	disable_fpga();
	fpga_power(true);	
	mdelay(10);
	enable_fpga();
#else
#error "Unknown FPGA!"
#endif
}

static void start_fpga_configuration(void)
{
        LOG_MSG("%s\n", __func__);
#if defined(CONFIG_FPGA_XILINX)
	gpio_request(GPIO_FPGA_INIT_n, "test-gpio");
	gpio_direction_output(GPIO_FPGA_INIT_n ,0);
	gpio_request(GPIO_FPGA_PROGRAM_n, "test-gpio2");	
	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,0);
	mdelay(10);
	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,1);
	udelay(10);
	gpio_direction_input(GPIO_FPGA_INIT_n);

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CONFIG_n, 0);
	gpio_direction_output(GPIO_FPGA_CE, 0);
	mdelay(10);
	gpio_direction_output(GPIO_FPGA_CONFIG_n , 1);

#endif
}

void setup_spi(void);

#ifdef CONFIG_FLIR_PLATFORM_EOCO
// tweaked eoco style fpga load - do we really need this?

static void request_fpga_config_pins(bool enable)
{
	if (enable) {
		gpio_request(GPIO_FPGA_CONFIG_n, "FPGA Config");
		gpio_request(GPIO_FPGA_STATUS_n, "FPGA Status");
		gpio_request(GPIO_FPGA_CE, "FPGA CE");
		gpio_request(GPIO_FPGA_CONF_DONE, "FPGA CONF_DONE");
	} else {
		gpio_free(GPIO_FPGA_CONFIG_n);
		gpio_free(GPIO_FPGA_STATUS_n);
		gpio_free(GPIO_FPGA_CE);
		gpio_free(GPIO_FPGA_CONF_DONE);
	}
}

int do_load_fpga(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int ret;
	disable_spi_bus();   //allow fpga to control spi bus
	request_fpga_config_pins(true);
	power_up_fpga();

	printf("Loading FPGA\n");

	/* Just start config, don't wait for it to complete */
	start_fpga_configuration();
	mdelay(1000);

	if (gpio_get_value(GPIO_FPGA_CONF_DONE) == 1) {
		printf("FPGA Loading done\n");
		ret = CMD_RET_SUCCESS;
	} else {
		printf("FPGA Loading failed\n");
		//FPGA did not load correctly from SPI flash
		//Disable FPGA loading to allow SPI access from linux
		gpio_direction_input(GPIO_FPGA_CE);
		gpio_direction_input(GPIO_FPGA_CONFIG_n);
		ret = CMD_RET_FAILURE;
	}

	request_fpga_config_pins(false);
	enable_spi_bus(); //take back control of spi bus

	return ret;
}

#else
// original style ec101 fpga load

static int do_load_fpga(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	enum { ST_RUN, ST_ERROR, ST_ABORT, ST_DONE } state;
	unsigned timeout;
	unsigned retry;

#ifdef CONFIG_SYS_USE_SPINOR
	/* FPGA already powered up by setup_spinor() above */
	if(!check_fpga_flash_header())
		return CMD_RET_FAILURE;
	disable_spi_bus();
	udelay(10);
#else
	power_up_fpga();
#endif
	printf("Loading FPGA ");

	if (argc < 2) {
		/* Just start config, don't wait for it to complete */
		start_fpga_configuration();
		printf("\n");
		return CMD_RET_SUCCESS;
	}

	retry = 2;
	state = ST_RUN;
	while (state == ST_RUN) {

		start_fpga_configuration();
		timeout = 50;

		while (state == ST_RUN) {
			mdelay(50);
			printf(".");
			if (ctrlc()) {
				printf(" abort\n");
				state = ST_ABORT;
				break;
			}
			if (gpio_get_value(GPIO_FPGA_CONF_DONE)==1) {
				printf(" done\n");
				state = ST_DONE;
				break;
			}
#ifdef CONFIG_FPGA_XILINX
			if (gpio_get_value(GPIO_FPGA_INIT_n)==0)
			{
				printf("\nFPGA CRC error, ");
				state = ST_ERROR;
				break;
			}
#endif
			if (--timeout == 0) {
				printf("\nFPGA configuration timeout, ");
				state = ST_ERROR;
				break;
			}
		}

		if (state == ST_DONE) {
			enable_fpga();
		} else {
			disable_fpga();
			if (state == ST_ERROR) {
				if (retry--) {
					printf("retrying\n");
					state = ST_RUN;
				} else {
					printf("giving up\n");
				}
			}
		}
	}

#ifdef CONFIG_SYS_USE_SPINOR
	enable_spi_bus();
#endif

	return (state == ST_DONE) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}
#endif

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(loadFPGA, 2, 0, do_load_fpga,
	   "Start load of main FPGA. Add 't' to wait for config to complete.",
	   "[t]"
	   );
#endif

U_BOOT_CMD(
    flir_loadfpga,	2,	0,	do_load_fpga,
    "Start load of main FPGA. Add 't' to wait for config to complete.",
    "[t]"
);
