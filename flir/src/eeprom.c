
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

    ret = i2c_set_bus_num(eeprom->i2c_bus);
    ret = i2c_read(chip, addr, 1, (uchar*)&hwrev, sizeof(hwrev));

    if(ret==0)
    {
	  eeprom->article  = simple_strtoul((char*)&hwrev.article[1], NULL, 10);
	  eeprom->revison = simple_strtoul(hwrev.revision, NULL, 10);
    }
    return ret;
}
