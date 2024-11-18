// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 FLIR Systems
 *
 * EC501 FPGA: Intel Altera 5CGXFC7B6M15I7N
 */
#include <spi.h>
#include <linux/mtd/spi-nor.h>
#include <linux/delay.h>
#include "../common/da9063.h"
#include "../common/da9063_regs.h"
#include "../common/fpga_ctrl.h"
#include "ec702.h"

#define GPIO_SPI1_SCLK     IMX_GPIO_NR(5, 22)
#define GPIO_SPI1_MOSI     IMX_GPIO_NR(5, 23)
#define GPIO_SPI1_MISO     IMX_GPIO_NR(5, 24)
#define GPIO_SPI1_CS       IMX_GPIO_NR(5, 28)

#define GPIO_FPGA_CONFIG_n  IMX_GPIO_NR(1,  7)
#define GPIO_FPGA_STATUS_n  IMX_GPIO_NR(1,  2)
#define GPIO_FPGA_CONF_DONE IMX_GPIO_NR(1,  8)
#define GPIO_FPGA_CE        IMX_GPIO_NR(5, 25)

#define SPI_FLASH_MAX_SIZE_BUF 32

extern struct spi_slave *slave;

static iomux_v3_cfg_t const ecspi1_pads[] = {
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

/**
 * ec702_fpga_power() - Enable or disable FPGA power
 *
 * Sequence:
 * - Read OTP revision
 * - Set CORE_SW (1V8D_FPGA)
 * - Set BUCKPRO (1V1D_FPGA)
 * - Set PERI_SW (1V2D_FPGA)
 * - Set BUCKMEM (2V5D_FPGA)
 * - Set LDO8    (3V15D_FPGA)
 */
#define VBPRO_1V1D 0x39 // 0.01V inc from 0.53V
#define VBMEM_2V5D 0x55 // 0.02V inc from 0.80V
static int ec702_fpga_power(bool enable)
{
	unsigned char conf_id;
	int ret;

	spi_claim_bus(slave);
	ret = pmic_read_reg(DA9063_REG_CHIP_CONFIG, &conf_id);
	if (ret) {
		log_err("Could not read PMIC ID registers\n");
		ret = -EIO;
		goto fpga_pwr_exit;
	}
	log_info("PMIC config rev 0x%02x\n", conf_id);

	ret = pmic_write_bitfield(DA9063_REG_BCORE1_CONT, DA9063_CORE_SW_EN,
				  enable ? DA9063_CORE_SW_EN : 0);
	if (ret) {
		log_err("Failed to set CORE_SW state\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_VBPRO_A, DA9063_VBUCK_MASK, VBPRO_1V1D);
	if (ret) {
		log_err("Failed to set BUCKPRO voltage\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_BPRO_CONT, DA9063_BUCK_EN,
				  enable ? DA9063_BUCK_EN : 0);
	if (ret) {
		log_err("Failed to set BUCKPRO state\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_BPERI_CONT, DA9063_PERI_SW_EN,
				  enable ? DA9063_PERI_SW_EN : 0);
	if (ret) {
		log_err("Failed to set PERI_SW state\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_VBMEM_A, DA9063_VBUCK_MASK, VBMEM_2V5D);
	if (ret) {
		log_err("Failed to set BUCKMEM voltage\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_BMEM_CONT, DA9063_BUCK_EN,
				  enable ? DA9063_BUCK_EN : 0);
	if (ret) {
		log_err("Failed to set BUCKMEM state\n");
		goto fpga_pwr_exit;
	}
	ret = pmic_write_bitfield(DA9063_REG_LDO8_CONT, DA9063_LDO_EN,
				  enable ? DA9063_LDO_EN : 0);
	if (ret) {
		log_err("Failed to set LDO8 state\n");
		goto fpga_pwr_exit;
	}

	// Power needs some time to stabilize, or subsequent
	// configuration will fail
	mdelay(200);

fpga_pwr_exit:
	spi_release_bus(slave);
	return ret;
}

static void ec702_fpga_set_ctrl(struct fpga_ctrl *fpga)
{
#if !IS_ENABLED(CONFIG_FPGA_ALTERA)
#error "ec702 needs to have CONFIG_FPGA_ALTERA set"
#endif

	fpga->pins.config_n = GPIO_FPGA_CONFIG_n;
	fpga->pins.status_n = GPIO_FPGA_STATUS_n;
	fpga->pins.done = GPIO_FPGA_CONF_DONE;
	fpga->pins.ce = GPIO_FPGA_CE;
	fpga_set_ops(fpga);
}

static int ec702_fpga_enable_power(struct fpga_ctrl *fpga)
{
	debug("%s\n",  __func__);
	return ec702_fpga_power(true);
}

static int ec702_fpga_request_flash_spi(struct fpga_ctrl *fpga)
{
	int ret = 0;

	debug("%s\n",  __func__);

	//Use as cpu spi bus
	ret = gpio_request(GPIO_SPI1_CS, "CS SPI1 0");
	if (ret)
		log_err("%s: gpio request failure %d\n",  __func__, ret);

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	ret = gpio_direction_output(GPIO_SPI1_CS, 1);
	if (ret)
		log_err("%s: gpio dir failure %d\n",  __func__, ret);

	return 0;
}

static int ec702_fpga_release_flash_spi(struct fpga_ctrl *fpga)
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
	ret += gpio_direction_input(GPIO_SPI1_CS);

	ret += gpio_free(GPIO_SPI1_SCLK);
	ret += gpio_free(GPIO_SPI1_MOSI);
	ret += gpio_free(GPIO_SPI1_MISO);
	ret += gpio_free(GPIO_SPI1_CS);

	if (ret) {
		log_err("%s: gpio failure\n",  __func__);
		return ret;
	}

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
 * Set up spi flash according to altera spec
 * Return negative on error
 */
static int ec702_fpga_init_spi_flash(struct fpga_ctrl *fpga)
{
	uchar hold_disable_mask = 0xef;
	uchar twelve_dummy_bits_mask = 0xcb;
	int ret = 0;

	(void)fpga;
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_WD_EVCR, &hold_disable_mask, 1);
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_MT_WR_ANY_REG, &twelve_dummy_bits_mask, 1);
	ret += spi_flash_cmd(SPINOR_OP_WREN, NULL, 0);
	ret += spi_flash_cmd(SPINOR_OP_EN4B, NULL, 0);

	return ret;
}

/**
 * @brief Revert (some) flash chip settings
 * Use 16 dummy bits after FAST READ
 * Disable 4B mode
 *
 * @return int negative on error
 */
static int ec702_fpga_uninit_spi_flash(struct fpga_ctrl *fpga)
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
	ops->fpga_request_flash_spi = ec702_fpga_request_flash_spi;
	ops->fpga_release_flash_spi = ec702_fpga_release_flash_spi;
	ops->fpga_enable_power = ec702_fpga_enable_power;
	ops->fpga_init_spi_flash = ec702_fpga_init_spi_flash;
	ops->fpga_uninit_spi_flash = ec702_fpga_uninit_spi_flash;
}

void fpga_init_ctrl(struct fpga_ctrl *fpga)
{
	debug("%s:\n", __func__);
	ec702_fpga_set_ctrl(fpga);

	fpga_set_board_ops(&fpga->board_ops);
}
