
#include <common.h>
#include <command.h>
#include <errno.h>
#include <i2c.h>
#include <malloc.h>

#include "../../../flir/include/eeprom.h"

/** Hardware version struct (in EEPROM) */
struct HWrevstruct
{
    char article[10];
    char serial[10];
    char revision[4];
    char moduleOffset[2];   /**< Address of a parent module, ModRevstruct */
    char moduleDevice[2];   /**< i2c device */
    char reserved[2];
};    /**< 32 bytes including checksum */

/*
reads from i2c bus
expects a structure HWrevstruct
at i2c_offset
returns
article and rev
*/
int get_eeprom_hwrev(struct Eeprom * eeprom)
{

    int ret = 0;
    struct HWrevstruct hwrev;
    int chip = (eeprom->i2c_address)>>1;
    int addr = eeprom->i2c_offset;

#if CONFIG_DM_I2C
    struct udevice *dev;
    struct udevice *bus;
    ret = uclass_get_device_by_seq(UCLASS_I2C, eeprom->i2c_bus, &bus);
    if (!ret)
        ret = i2c_get_chip(bus, chip, 1, &dev);
    if (!ret)
        ret = dm_i2c_read(dev, addr, (uchar *)&hwrev, sizeof(hwrev));
#else
    ret = i2c_set_bus_num(eeprom->i2c_bus);
    ret = i2c_read(chip, addr, 1, (uchar*)&hwrev, sizeof(hwrev));
#endif

    if(ret==0)
    {
	  eeprom->article  = simple_strtoul((char*)&hwrev.article[1], NULL, 10);
	  eeprom->revison = simple_strtoul(hwrev.revision, NULL, 10);
    }
    return ret;
}
