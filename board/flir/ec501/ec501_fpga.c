// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 FLIR Systems
 *
 * EC501 FPGA: Altera 5CGXF7B6M157
 *
 * Configuration MSEL[4:0] = [10011] -- Active serial mode
 * pin             CPU config
 * SPI1_SCLK_FPGA  hiZ
 * SPI1_CS1_n      hiZ
 * SPI1_MOSI       hiZ
 * SPI1_MISO       hiZ
 * FPGA_CE_n       output, must be low during config, set high to release spi
 * CONFIG_n        output, config triggered by L->H transition
 * STATUS_n        input
 * CONF_DONE       input
 */
#include <spi.h>
#include <linux/mtd/spi-nor.h>
#include <linux/delay.h>
#include "ec501.h"
#include "../common/da9063.h"
#include "../common/da9063_regs.h"
#include "../common/fpga_ctrl.h"
#include "../common/cmd_loadfpga.h"
#include "../common/da9063_regs.h"

#define GPIO_SPI1_SCLK     IMX_GPIO_NR(5, 22)
#define GPIO_SPI1_MOSI     IMX_GPIO_NR(5, 23)
#define GPIO_SPI1_MISO     IMX_GPIO_NR(5, 24)
#define GPIO_SPI1_CS       IMX_GPIO_NR(5, 28)

#define GPIO_FPGA_CONFIG_n  IMX_GPIO_NR(5, 25)
#define GPIO_FPGA_STATUS_n  IMX_GPIO_NR(5, 26)
#define GPIO_FPGA_CONF_DONE IMX_GPIO_NR(5, 27)
#define GPIO_FPGA_CE        IMX_GPIO_NR(4, 10)

#define SPI_FLASH_MAX_SIZE_BUF 32

static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const no_ecspi1_pads[] = {
	MX6_PAD_CSI0_DAT4__GPIO5_IO22  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT5__GPIO5_IO23  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT6__GPIO5_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

extern struct spi_slave *slave;

int fpga_power(bool enable)
{
	//Duplo VBUCKMEM, CORE_SW_S, PERI_SWS, LDO8, LDO10
	int ret;
	unsigned char conf_id;

	debug("%s(%s)\n",  __func__, enable ? "enable" : "disable");

	ret = spi_claim_bus(slave);
	if (ret) {
		printf("%s: Failed to claim spi bus\n", __func__);
		return ret;
	}

	if (pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id)) {
		printf("Could not read PMIC ID registers\n");
		spi_release_bus(slave);
		return -1;
	}

	// CORE_SW_EN  (1V8_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_BCORE1_CONT,
				  DA9063_CORE_SW_EN,
				  enable ? DA9063_CORE_SW_EN : 0);
	if (ret) {
		printf("Failed to enable 1V8_FPGA\n");
		return ret;
	}

	// BUCK_MEM    (1V1_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_BMEM_CONT,
				  DA9063_BUCK_EN,
				  enable ? DA9063_BUCK_EN : 0);
	if (ret) {
		printf("Failed to enable 1V1_FPGA\n");
		return ret;
	}

	// PERI_SW_EN    (1V2_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_BPERI_CONT,
				  DA9063_PERI_SW_EN,
				  enable ? DA9063_PERI_SW_EN : 0);
	if (ret) {
		printf("Failed to enable 1V2_FPGA\n");
		return ret;
	}
	// LDO10_EN          (2V5_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_LDO10_CONT,
				  DA9063_LDO_EN,
				  enable ? DA9063_LDO_EN : 0);
	if (ret) {
		printf("Failed to enable 2V5_FPGA\n");
		return ret;
	}
	// LDO8_EN          (3V15_FPGA)
	ret = pmic_write_bitfield(DA9063_REG_LDO8_CONT,
				  DA9063_LDO_EN,
				  enable ? DA9063_LDO_EN : 0);
	if (ret) {
		printf("Failed to enable 3V15_FPGA\n");
		return ret;
	}

	spi_release_bus(slave);

	return ret;
}

static void ec501_fpga_set_ctrl(struct fpga_ctrl *fpga)
{
#if !IS_ENABLED(CONFIG_FPGA_ALTERA)
#error "ec501 needs to have CONFIG_FPGA_ALTERA set"
#endif
	debug("%s\n",  __func__);
	fpga->pins.config_n = GPIO_FPGA_CONFIG_n;
	fpga->pins.status_n = GPIO_FPGA_STATUS_n;
	fpga->pins.done = GPIO_FPGA_CONF_DONE;
	fpga->pins.ce = GPIO_FPGA_CE;

	fpga_set_ops(fpga);
}

static int ec501_fpga_enable_power(struct fpga_ctrl *fpga)
{
	debug("%s\n",  __func__);
	return fpga_power(true);
}

static int ec501_fpga_request_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//Use as cpu spi bus
	ret = gpio_request(GPIO_SPI1_CS, "spi-1-cs");
	if (ret)
		log_err("%s: gpio request failure %d\n",  __func__, ret);

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	ret = gpio_direction_output(GPIO_SPI1_CS, 1);
	if (ret)
		log_err("%s: gpio dir failure %d\n",  __func__, ret);

	return ret;
}

static int ec501_fpga_release_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//cpu spi bus conflicts with fpga spi bus, disable cpu bus
	imx_iomux_v3_setup_multiple_pads(no_ecspi1_pads,
					 ARRAY_SIZE(no_ecspi1_pads));

	ret += gpio_request(GPIO_SPI1_SCLK, "spi-1-clk");
	ret += gpio_request(GPIO_SPI1_MOSI, "spi-1-mosi");
	ret += gpio_request(GPIO_SPI1_MISO, "spi-1-miso");
	ret += gpio_direction_input(GPIO_SPI1_SCLK);
	ret += gpio_direction_input(GPIO_SPI1_MOSI);
	ret += gpio_direction_input(GPIO_SPI1_MISO);
	ret += gpio_direction_output(GPIO_SPI1_CS, 1);
	ret += gpio_direction_input(GPIO_SPI1_CS);

	ret += gpio_free(GPIO_SPI1_SCLK);
	ret += gpio_free(GPIO_SPI1_MOSI);
	ret += gpio_free(GPIO_SPI1_MISO);
	ret += gpio_free(GPIO_SPI1_CS);

	if (ret)
		log_err("%s: gpio failure\n",  __func__);

	return 0;
}

static int do_spi_xfer(int bus, int cs, int freq, int mode, uchar *dout, int len)
{
	struct spi_slave *slave;
	int ret = 0;
	int bitlen = (1 + len) * 8;
	uchar din[SPI_FLASH_MAX_SIZE_BUF];

	if (CONFIG_IS_ENABLED(DM_SPI)) {
		char name[30], *str;
		struct udevice *dev;

		snprintf(name, sizeof(name), "generic_%d:%d", bus, cs);
		str = strdup(name);
		if (!str)
			return -ENOMEM;
		ret = spi_get_bus_and_cs(bus, cs, freq, mode, "spi_generic_drv",
					 str, &dev, &slave);
		if (ret)
			return ret;
	} else {
		slave = spi_setup_slave(bus, cs, freq, mode);
		if (!slave) {
			printf("Invalid device %d:%d\n", bus, cs);
			return -EINVAL;
		}
	}

	ret = spi_claim_bus(slave);
	if (ret)
		goto done;

	ret = spi_xfer(slave, bitlen, dout, din,
		       SPI_XFER_BEGIN | SPI_XFER_END);
	if (!CONFIG_IS_ENABLED(DM_SPI)) {
		/* We don't get an error code in this case */
		if (ret)
			ret = -EIO;
	}

done:
	spi_release_bus(slave);
	if (!CONFIG_IS_ENABLED(DM_SPI))
		spi_free_slave(slave);

	return ret;
}

/**
 * @brief Write a command to the spi flash
 *
 * @param cmd
 * @param dout the data to be written, null if no parameters to cmd
 * @param len length of dout
 * @return int
 */
static int spi_flash_cmd(uchar cmd, uchar *dout, size_t len)
{
	uchar buf[SPI_FLASH_MAX_SIZE_BUF] = {cmd};
	unsigned int bus = CONFIG_DEFAULT_SPI_BUS;
	unsigned int cs = CONFIG_SF_DEFAULT_CS;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
	unsigned int freq = CONFIG_SF_DEFAULT_SPEED;

	if ((len + 1 >= SPI_FLASH_MAX_SIZE_BUF) || (len > 0 && !dout))
		return -EINVAL;

	if (len > 0)
		memcpy(&buf[1], dout, len);

	return do_spi_xfer(bus, cs, freq, mode, buf, len);
}

/**
 * @brief Set up spi flash according to altera spec
 * Disable HOLD pin functionality
 * Use 12 dummy bits after FAST READ
 * Enable 4B mode
 *
 * @return int negative on error
 */
static int ec501_fpga_init_spi_flash(struct fpga_ctrl *fpga)
{
	u8 hold_disable_mask = 0xef;
	u8 twelve_dummy_bits_mask = 0xcb;
	int ret = 0;

	(void)fpga;
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_WD_EVCR, &hold_disable_mask, 1);
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_MT_WR_ANY_REG, &twelve_dummy_bits_mask, 1);
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_EN4B, NULL, 0);
	if (ret)
		log_err("Failed to configure the SPI Flash\n");

	return ret;
}

/**
 * @brief Revert (some) flash chip settings
 * Use 16 dummy bits after FAST READ
 * Disable 4B mode
 *
 * @return int negative on error
 */
static int ec501_fpga_uninit_spi_flash(struct fpga_ctrl *fpga)
{
	u8 sixteen_dummy_bits_mask = 0xfb;
	int ret = 0;

	(void)fpga;
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_MT_WR_ANY_REG, &sixteen_dummy_bits_mask, 1);
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_EX4B, NULL, 0);
	if (ret)
		log_err("Failed to de-configure the SPI Flash\n");

	return ret;
}

static void fpga_set_board_ops(struct fpga_board_ops *ops)
{
	debug("%s\n",  __func__);
	ops->fpga_request_flash_spi = ec501_fpga_request_flash_spi;
	ops->fpga_release_flash_spi = ec501_fpga_release_flash_spi;
	ops->fpga_enable_power = ec501_fpga_enable_power;
	ops->fpga_init_spi_flash = ec501_fpga_init_spi_flash;
	ops->fpga_uninit_spi_flash = ec501_fpga_uninit_spi_flash;
}

void fpga_init_ctrl(struct fpga_ctrl *fpga)
{
	debug("%s\n",  __func__);
	ec501_fpga_set_ctrl(fpga);
	fpga_set_board_ops(&fpga->board_ops);
}
