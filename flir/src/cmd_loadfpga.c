#include <common.h>
#include <command.h>
#include <asm/errno.h>
#include <da9063.h>
#include <cmd_loadfpga.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>

extern int fpga_power(bool enable);
static void power_up_fpga(void);


//FPGA Configuration is documented in 
// ALtera: CV-52007 (2017.09.19)
// Xilinx: UG470 (v1.11, 2016.09.27)
// Xilinx: xapp586 (v1.3 2016.10.28)
// (Xilinx XAPP583, (V1.0 2012.05.31))
//
//FPGA Configuration pins:
//
// EC101  FPGA: Xilinx XC7A100T 
// Configuration MSEL[2:0] = [001] -- Master SPI mode
//pin             CPU config
//SPI1_SCLK_QSPI   hiZ
//SPI1_CS1_n       hiZ
//SPI1_MOSI        hiZ
//SPI1_MISO        hiZ
//SPI1_CS1_n       hiZ
//PROGRAM_n        output
//INIT_n           input/output (do not drive high)
//CONF_DONE        input
//
//Program (bar)
//  Active-Low reset to configuration logic. When
//  PROGRAM_B is pulsed Low, the FPGA configuration is
//  cleared and a new configuration sequence is initiated.
//  Configuration reset initiated upon falling edge, and
//  configuration (i.e. programming) sequence begins upon
//  the following rising edge.
//  Connect PROGRAM_B to an external 4.7 k pull-up
//  resistor to VCCO_0 to ensure a stable High input, and
//  recommend push-button to GND to enable manual
//  configuration reset.
//  Note: Holding PROGRAM_B Low from power-on does not
//  keep the FPGA configuration in reset. Instead, use INIT_B
//  to delay the power-on configuration sequence.
//Initialization (bar)
//  Active-Low FPGA initialization pin or configuration
//  error signal. The FPGA drives this pin Low when the
//  FPGA is in a configuration reset state, when the FPGA is
//  initializing (clearing) its configuration memory, or when
//  the FPGA has detected a configuration error. Upon
//  completing the FPGA initialization process, INIT_B is
//  released to high-impedance at which time an external
//  resistor is expected to pull INIT_B High. INIT_B can
//  externally be held Low during power-up to stall the
//  power-on configuration sequence at the end of the
//  initialization process. When a High is detected at the
//  INIT_B input after the initialization process, the FPGA
//  proceeds with the remainder of the configuration
//  sequence dictated by the M[2:0] pin settings.
//  Connect INIT_B to a 4.7 k pull-up resistor to VCCO_0
//  to ensure clean Low-to-High transitions.
//
// EC501 FPGA: Altera 5CGXF7B6M157
// Configuration MSEL[4:0] = [10011] -- Active serial mode
//pin             CPU config
//SPI1_SCLK_FPGA  hiZ
//SPI1_CS1_n      hiZ
//SPI1_MOSI       hiZ 
//SPI1_MISO       hiZ
//FPGA_CE_n       output, must be low during config, set high to release spi
//CONFIG_n        output, config triggered by L->H transition
//STATUS_n        input
//CONF_DONE       input


#ifdef CONFIG_SYS_USE_SPINOR

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define HEADER_LENGTH  65536
static struct spi_flash *flash;


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

static void enable_spi_bus(void)
{
	//restore cpu spi bus
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	gpio_direction_output(GPIO_SPI1_CS,1);
}

static void disable_spi_bus(void)
{
	//cpu spi bus conflicts with fpga spi bus, disable cpu bus
	imx_iomux_v3_setup_multiple_pads(no_ecspi1_pads,
					 ARRAY_SIZE(no_ecspi1_pads));
	gpio_direction_input(GPIO_SPI1_SCLK);
	gpio_direction_input(GPIO_SPI1_MOSI);
	gpio_direction_input(GPIO_SPI1_MISO);
	gpio_direction_output(GPIO_SPI1_CS,1);
	gpio_direction_input(GPIO_SPI1_CS);
}

void setup_spinor(void)
{
	power_up_fpga();
	enable_spi_bus();
	udelay(10);
	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
				CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
}

#endif /* CONFIG_SYS_USE_SPINOR */


static void power_up_fpga(void)
{
#if defined(CONFIG_FPGA_XILINX)

	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,0);
	gpio_direction_output(GPIO_FPGA_INIT_n ,0);
	gpio_direction_input(GPIO_FPGA_CONF_DONE);
	fpga_power(true);
	mdelay(10);

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CONFIG_n, 0);
	gpio_direction_input(GPIO_FPGA_CONF_DONE);
	gpio_direction_input(GPIO_FPGA_STATUS_n);
	gpio_direction_input(GPIO_FPGA_CE);
	fpga_power(true);
	mdelay(10);
	gpio_direction_output(GPIO_FPGA_CE, 1);

#else
#error "Unknown FPGA!"
#endif
}

static void disable_fpga(void)
{
#if defined(CONFIG_FPGA_XILINX)

	gpio_direction_output(GPIO_FPGA_PROGRAM_n ,0);
	gpio_direction_output(GPIO_FPGA_INIT_n ,0);

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CONFIG_n , 0);
	gpio_direction_output(GPIO_FPGA_CE, 1);

#endif
}

static void start_fpga_configuration(void)
{
#if defined(CONFIG_FPGA_XILINX)

	gpio_direction_output(GPIO_FPGA_INIT_n ,0);
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

static void enable_fpga(void)
{
#if defined(CONFIG_FPGA_XILINX)

	/* n/a */

#elif defined(CONFIG_FPGA_ALTERA)

	gpio_direction_output(GPIO_FPGA_CE, 1);
	udelay(10);

#endif
}

static int do_load_fpga(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
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

U_BOOT_CMD(
    loadFPGA,	2,	0,	do_load_fpga,
    "Start load of main FPGA. Add 't' to wait for config to complete.",
    "[t]"
);
