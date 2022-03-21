#include <common.h>
#include <command.h>
#include <linux/delay.h>
#include <i2c.h>
#include <asm/io.h>

#include "../../../flir/include/cmd_checkcharger.h"

static int do_check_charger(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    printf("Check allowed charging current\n");
    unsigned long tmp, tmp2;
    unsigned char buf[100];
    printf("USB Check charger I2C disabled...");
    /* i2c_set_bus_num(2); */

    // Enable the vdd3p0 curret limiter
    // Set PMU_REG_3P0.ENABLE_ILIMIT = 1'b1
    // and PMU_REG_3P0.ENABLE_LINREG = 1' b1

    tmp = readl(PMU_REG_3P0);
    writel(tmp | 0x5, PMU_REG_3P0);

    // USBn_VBUS_DETECT_STAT
    // VBUSVALID=1'b1?
    tmp = readl(USB_ANALOG_USB1_VBUS_DETECT_STAT);
    if ((tmp & USB_ANALOG_USB1_VBUS_DETECT_STAT_VBUSVALID) == 0){
	    // VBUS is not valid
	    // Turn off the charger detector
	    // (set EN_B and CHK_CHRG_B to 1)
	    // In register USBn_VBUS_DETECT
	    // Continue boot up
	    printf("Vbus not detected!!\n");
	    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT1);
	    writel(tmp | (0x3 << 19), USB_ANALOG_USB1_CHRG_DETECT1);  //CHK_CHRG_B=1, EN_B=1
	    return 0;
    }

    // VBUS is valid
    // Set CHK_CHRG_B = 1'b1
    // And CHK_CONTACT = 1'b1

    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT1);
    writel(tmp | (0x3 << 18), USB_ANALOG_USB1_CHRG_DETECT1);

    // USB plug contacted ?
    // (monitor the PLUG_CONTACT bit)

    mdelay(100);
    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT_STAT);
    if (!(tmp & 0x1)){
	    // VBUS is coming from a dedicated power supply.
	    // No USB actions required.
	    // Continue boot up
	    printf("USB VBUS power from external source...\n");
	    return 0;
    }

    // Set CHK_CONTACT=1'b0
    // And CHK_CHRG_B = 1' b0

    printf("USB plug connected\n");
    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT1);
    writel(tmp ^ (0x3 << 18), USB_ANALOG_USB1_CHRG_DETECT1);  //CHK_CONTACT=0, CHK_CHRG_B=0

    // Wait for > 40ms

    mdelay(100);

    // Is it a charger ?
    // (monitor the CHRG_DETECTED bit)

    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT_STAT); //Is it a charger
    if (!(tmp & 0x2)){

	    // Turn off the       (set EN_B and charger detector   CHK_CHRG_B to 1)
	    // Initiliaze system and USB (can only draw 100mA current)
	    // pull DP high to enumerate
	    // Continue boot up

	    printf("Not a charger.... (100mA)\n");
	    /* i2c_read(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
	    /* buf[0]=(buf[0] & 0xf8)| 0x1; //100mA current limit */
	    /* i2c_write(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
	    return 100;
    }


    printf("USB charger detected...\n");

    // Turn off the charger detector (set EN_B and CHK_CHRG_B to 1)
    // Wait for > 40ms
    // Initiliaze system and USB (Recommend to draw less than 900mA)
    // (can draw high current 1.5A)
    // Disable the vdd3p0 curret limiter
    // Set PMU_REG_3P0.
    // ENABLE_ILIMIT = 1'b0
    // Pull up DP

    tmp = readl(USB_ANALOG_USB1_CHRG_DETECT1);
    writel(tmp | (0x3 << 19), USB_ANALOG_USB1_CHRG_DETECT1);  //CHK_CHRG_B=1, EN_B=1

    tmp = readl(USBPHY1_DEBUG);
    writel((tmp & 0xffffffc3), USBPHY1_DEBUG);

    tmp = readl(USBPHY1_CTRL);
    writel((tmp | 1 << 4), USBPHY1_CTRL);

    mdelay(100);

    //Is DM high ?
    //(monitor the
    //DM_STATE bit)

    tmp2 = readl(USB_ANALOG_USB1_CHRG_DETECT_STAT);
    if (tmp2 & 0x1 << 2){

	    //Dedicated Charger (can draw 1.8A)
	    //Continue boot up

	    printf("It is a dedicated charging port (1500mA)\n");
	    writel(tmp, USB_ANALOG_USB1_CHRG_DETECT1);
	    /* i2c_read(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
	    /* buf[0]=(buf[0] & 0xf8) | 0x5;  //1500mA current limit */
	    /* i2c_write(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
	    return 1500;
    }

    // Charging Downstream Port
    // Continue boot up (can draw 900mA)

    writel(tmp, USB_ANALOG_USB1_CHRG_DETECT1);
    printf("It is a charging downstream port (900mA)\n");
    /* i2c_read(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
    /* buf[0]=(buf[0] & 0xf8)| 0x3; //900 mA current limit */
    /* i2c_write(BQ24298_I2C_ADDR, 0x0, 1, buf, 1); */
    return 900;
}



#ifdef CONFIG_FLIR_NEW_COMMAND_STYLE
U_BOOT_CMD(
	   checkCharger, 1, 0, do_check_charger,
	   "",
	   ""
);
#endif

U_BOOT_CMD(
	   flir_checkcharger, 1, 0, do_check_charger,
	   "",
	   ""
);
