// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#include <common.h>
#include <asm/io.h>
#include <i2c.h>
#include <asm/arch/mx7ulp-pins.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include "pf1550.h"
#include "linux/delay.h"

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201) || CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
#define PMIC_WDOG_GPIO	IMX_GPIO_NR(1, 14)
static iomux_cfg_t const pmic_wdog_pad[] = {
	MX7ULP_PAD_PTA14__PTA14 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
#if (CONFIG_IS_ENABLED(DM_PCA953X))
#define PMIC_WDOG_GPIO	IMX_GPIO_NR(3, 0+8)
#else
#define PMIC_WDOG_GPIO	IMX_GPIO_NR(3, 0)
#endif
static iomux_cfg_t const pmic_wdog_pad[] = {
	MX7ULP_PAD_PTC0__PTC0 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif


int pf1550_write_reg(int reg, u8 val)
{
	struct udevice *dev;
	int ret = i2c_get_chip_for_busnum(5, 0x8, 1, &dev);

	if (ret) {
		printf("Can not find pmic: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_write(dev, reg, &val, 1);
	if (ret) {
		printf("Failed to write pmic: %d\n", ret);
		return ret;
	}

	return ret;
}

int pf1550_read_reg(int reg, u8 *val)
{
	struct udevice *dev;
	int ret = i2c_get_chip_for_busnum(5, 0x8, 1, &dev);

	if (ret) {
		printf("Can not find pmic: %d\n", ret);
		return ret;
	}

	ret = dm_i2c_read(dev, reg, val, 1);
	if (ret) {
		printf("Failed to read pmic: %d\n", ret);
		return ret;
	}

	return ret;
}

int get_usb_cable_state(void)
{
	u8 chg_int_ok = 0;

	pf1550_read_reg(PF1550_CHARG_REG_VBUS_SNS, &chg_int_ok);

	if (chg_int_ok & PF1550_VBUS_VALID)
		return 1;

	return 0;
}

int get_onoff_key(void)
{
	u8 onkey_int = 0;

	pf1550_read_reg(PF1550_PMIC_REG_ONKEY_INT_STAT0, &onkey_int);
	pf1550_write_reg(PF1550_PMIC_REG_ONKEY_INT_STAT0, onkey_int);

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
	if (onkey_int & ONKEY_IRQ_1SI)
		return 1;
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201) || CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
	if (onkey_int & ONKEY_IRQ_PUSHI)
		return 1;
#endif

	return 0;
}

void pmic_goto_core_off(bool enable)
{
	u8 buf = enable ? 2 : 0;
	//set go to core off mode
	pf1550_write_reg(PF1550_PMIC_REG_PWRCTRL3, buf);
}

void power_off(void)
{
	printf("Powering off....\n");
	//set go to core off mode
	pmic_goto_core_off(true);
	//set watchdog signal low
	gpio_direction_output(PMIC_WDOG_GPIO, 0);

	while (1) {
		mdelay(1000);
		printf("Still on\n");
	}
}

void reboot(void)
{
	printf("Reboot....\n");
	//set go to core off mode
	pmic_goto_core_off(false);
	//set watchdog signal low
	gpio_direction_output(PMIC_WDOG_GPIO, 0);

	while (1) {
	}
}

void init_pf1550_pmic(void)
{
	u8 curr_pwr_ctrl0;
	u8 new_pwr_ctrl0;

	//set Carger operation to charger=off, linear=on
	pf1550_write_reg(PF1550_CHARG_REG_CHG_OPER, CHARGER_OFF_LINEAR_ON);
	//set THM_CNFG=NO_THERMISTOR_CONTROL REGTEMP=95C THM_COLD=0C THM_HOT=55C TMP_FB_EN=1
	pf1550_write_reg(PF1550_CHARG_REG_THM_REG_CNFG, 0xa4);
	//set JEITA THM_WARM=50C THM_COOL=10C CV_ADJ=60mV CC_ADJ=25%
	pf1550_write_reg(PF1550_CHARG_REG_THM_ADJ_SETTING, 0x03);
	//set charger current to 1A PRECHGLB_THRS=2.8V
	pf1550_write_reg(PF1550_CHARG_REG_CHG_CURR_CNFG, 0x12);

	//set time to press off button before triggering a PMIC reset
	pf1550_read_reg(PF1550_PMIC_REG_PWRCTRL0, &curr_pwr_ctrl0);
#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC401W))
	new_pwr_ctrl0 = (curr_pwr_ctrl0 & ~PF1550_PMIC_REG_PWRCTRL0_TGRESET_MASK)
			 | PF1550_PMIC_REG_PWRCTRL0_TGRESET_16S;
#elif (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC201) || CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
	new_pwr_ctrl0 = (curr_pwr_ctrl0 & ~PF1550_PMIC_REG_PWRCTRL0_TGRESET_MASK)
			 | PF1550_PMIC_REG_PWRCTRL0_TGRESET_8S;
#endif
	pf1550_write_reg(PF1550_PMIC_REG_PWRCTRL0, new_pwr_ctrl0);

#if (CONFIG_IS_ENABLED(TARGET_MX7ULP_EC302))
	//enable charger and set charging voltage to 3.1V
	pf1550_write_reg(PF1550_PMIC_REG_COINCELL_CONTROL, 0x1D);
	//set default value to VSNVS register
	pf1550_write_reg(PF1550_PMIC_REG_VSNVS_CTRL, 0x00);
#endif

	mx7ulp_iomux_setup_multiple_pads(pmic_wdog_pad, ARRAY_SIZE(pmic_wdog_pad));
	gpio_request(PMIC_WDOG_GPIO, "pmic_wdog");
}

void pf1550_thm_ok_toogle_charging(void)
{
	u8 chg_int_ok;
	static u8 chg_oper;
	u8 chg_oper_reg;
	u8 thm_ok;
	u8 vbus_ok;
	int res = 0;

	res = pf1550_read_reg(PF1550_CHARG_REG_CHG_INT_OK, &chg_int_ok);
	if (res)
		return;

	vbus_ok = chg_int_ok & PF1550_CHG_INT_OK_VBUS_OK;
	thm_ok = chg_int_ok & PF1550_CHG_INT_OK_THM_OK;
	vbus_ok >>= PF1550_CHG_INT_OK_VBUS_OK_SHIFT;
	thm_ok >>= PF1550_CHG_INT_OK_THM_OK_SHIFT;

	if (thm_ok && vbus_ok) {
		/* Inside THM range and VBUS is OK, enable charging. */
		if (chg_oper != CHARGER_ON_LINEAR_ON) {
			printf("Enable charging (THM_OK: %d, VBUS_OK: %d).\n", thm_ok, vbus_ok);
			chg_oper = CHARGER_ON_LINEAR_ON;
		}
	} else {
		/* Outside of THM range or VBUS is not OK, disable charging. */
		if (chg_oper != CHARGER_OFF_LINEAR_ON) {
			printf("Disable charging (THM_OK: %d, VBUS_OK: %d).\n", thm_ok, vbus_ok);
			chg_oper = CHARGER_OFF_LINEAR_ON;
		}
	}

	/* Apply charge operation mode if needed */
	res = pf1550_read_reg(PF1550_CHARG_REG_CHG_OPER, &chg_oper_reg);
	if (res)
		return;

	if ((chg_oper_reg & PF1550_CHARG_REG_CHG_OPER_CHG_OPER_MASK) != chg_oper) {
		chg_oper_reg = (chg_oper_reg & ~PF1550_CHARG_REG_CHG_OPER_CHG_OPER_MASK) | chg_oper;
		pf1550_write_reg(PF1550_CHARG_REG_CHG_OPER, chg_oper_reg);
	}
}

void set_charging_current(int mA)
{
	u8 ilim = _10ma << VBUS_LIN_ILIM_SHIFT;

	if (mA >= 1500)
		ilim = _1500ma << VBUS_LIN_ILIM_SHIFT;
	else if (mA >= 1000)
		ilim = _1000ma << VBUS_LIN_ILIM_SHIFT;
	else if (mA >= 500)
		ilim = _500ma  << VBUS_LIN_ILIM_SHIFT;
	else if (mA >= 100)
		ilim = _100ma << VBUS_LIN_ILIM_SHIFT;
	else
		ilim = _10ma << VBUS_LIN_ILIM_SHIFT;

	pf1550_write_reg(PF1550_CHARG_REG_VBUS_INLIM_CNFG, ilim);

	// turn on charging if within allowed thermal range
	pf1550_thm_ok_toogle_charging();
}
