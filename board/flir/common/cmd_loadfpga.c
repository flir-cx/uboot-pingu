// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems.
 */

#include <common.h>
#include <command.h>
#include <errno.h>
#include <linux/delay.h>
#include <spi_flash.h>

#include "fpga_ctrl.h"
#include "cmd_loadfpga.h"

#define HEADER_LENGTH  65536
static struct spi_flash *flash;

/**
 * @brief Powers up the fpga and holds it in reset or config
 *
 * This is needed to put power to both the spi flash and the
 * fpga since they share 3V15 in in our design
 *
 * @param fpga
 * @return int
 */
static int fpga_power_and_hold(struct fpga_ctrl *fpga)
{
	int ret;

	ret = fpga_request_pinctl(fpga);
	if (ret)
		return ret;
	ret = fpga_hold_in_config(fpga);
	if (ret)
		return ret;
	ret = fpga_disable(fpga);
	if (ret)
		return ret;
	ret = fpga_enable_power(fpga);
	mdelay(10);

	return ret;
}

/**
 * @brief Set the up spinor flash, this is called from the board file
 * to secure that the spi flash is powered and pins can be used as spi
 * from the cpu. Since the spi bus of the flash is shared between the
 * cpu and the fpga, the spi bus is owned by the cpu after a call to
 * this function.
 *
 */
void setup_spinor(void)
{
#if defined CONFIG_SYS_USE_SPINOR
	struct fpga_ctrl fpga;

	debug("%s:\n", __func__);
	fpga_init_ctrl(&fpga);
	fpga_power_and_hold(&fpga);
	fpga_request_flash_spi(&fpga);
	udelay(10);

#if 0
	/* disabled for now, see BC-150 */
	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
				CONFIG_SF_DEFAULT_CS,
				CONFIG_SF_DEFAULT_SPEED,
				CONFIG_SF_DEFAULT_MODE);
#endif
#endif

}

/**
 * @brief Looks for a magic sequence in flash to identify our fpga
 *
 * @return int 0 on success, -ve otherwise
 */
static int check_fpga_flash_header(void)
{
	char buf[6] = {0};
	int offset = 0;
	int ret;

	if (!flash) {
		log_info("SPI FLASH probe failed, keep going anyway\n");
		return 0; // try to load fpga anyway
	}

	offset = flash->size - HEADER_LENGTH;
	ret = spi_flash_read(flash, offset, 6, buf);
	if (ret) {
		log_err("%s: Flash read failed (%d)\n", __func__, ret);
		return ret;
	}
	if (memcmp(buf, "FLIR", 4)) {
		log_err("Error: FPGA header missing\n");
		return -EBADE;
	}
	log_info("FPGA header OK\n");

	//reset address to 0
	spi_flash_read(flash, 0, 6, buf);

	return 0;
}

/**
 * @brief Releases a the fpga to start loading its code from its hold state
 *
 * @param ms_timeout Give up after timeout
 * @return int CMD_RET_SUCCESS on success, CMD_RET_FAILURE otherwise
 */
static int load_fpga(uint ms_timeout)
{
	const uint poll_rate = 10; // ms
	uint timeout_laps = ms_timeout / poll_rate;
	int i;
	struct fpga_ctrl fpga;
	int ret;

	fpga_zero_ctrl(&fpga);
	fpga_init_ctrl(&fpga);
#if defined CONFIG_SYS_USE_SPINOR
	/*
	 * FPGA already powered up by setup_spinor() above,
	 * called from board_late_init
	 */
	if (check_fpga_flash_header())
		return CMD_RET_FAILURE;

	fpga_init_spi_flash(&fpga);
	fpga_release_flash_spi(&fpga);
	udelay(10);
#else
	fpga_enable_power(&fpga);
	fpga_init_spi_flash(&fpga);
#endif
	log_info("Loading FPGA ");
	fpga_start_config(&fpga);

	for (i = 0; i < timeout_laps; i++) {
		mdelay(poll_rate);
		ret = fpga_poll_config_status(&fpga);
		if (ret != 0)
			break;

		if (i % 5 == 0)
			log_info(".");
	}
	if (ret > 0) {
		log_info(" done (took %u ms)\n", poll_rate * i);
		fpga_enable(&fpga);
	} else if (ret < 0) {
		log_err("\nFPGA CRC error, giving up\n");
		fpga_disable(&fpga);
	} else {
		log_err("\nFPGA timeout reached, failed to config FPGA\n");
		fpga_disable(&fpga);
	}

#if defined CONFIG_SYS_USE_SPINOR
	fpga_request_flash_spi(&fpga);
#endif
	return (ret > 0) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}


static int do_load_fpga(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned timeout;

	if (argc < 2)
		timeout = 2500;
	else if (argc == 2 && argv[1][0] == 't')
		timeout = 2500; //default time for t
	else if (argc == 3 && argv[1][0] == 't')
		timeout = simple_strtol(argv[2], NULL, 10);
	else
		return CMD_RET_FAILURE;

	return load_fpga(timeout);
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(loadFPGA, 3, 0, do_load_fpga,
    "Start load of main FPGA.\n \
          Add 't' to wait 2500 ms for config to complete.\n \
          Add 't 1000' to wait for 1s for config to complete.",
    "[t [timeout_ms]]"
	   );
#endif

U_BOOT_CMD(
    flir_loadfpga,	3,	0,	do_load_fpga,
    "Start load of main FPGA.\n \
               Add 't' to wait 2500 ms for config to complete.\n \
               Add 't 1000' to wait for 1s for config to complete.",
    "[t [timeout_ms]]"
);
