/*
 * Copyright (C) 2015 FLIR Systems.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */



#include <common.h>
#include <command.h>
#include <errno.h>
#include <i2c.h>
#include <malloc.h>
#include "../../../flir/include/cmd_updatefdteeprom.h"
#include <fdt_support.h>
#include "eeprom.h"

/*
* every computer boards contains an eeprom
* describing the article and revision number of that board,
* this is later passed to the kernel in a device tree
*/


struct Eeprom_list eeprom_list = {
	.list		= LIST_HEAD_INIT(eeprom_list.list),
};



/*will patch a fdt blob with \
 *  /boards/x/article
 *  /boards/x/rev
 *  entries
*/

void patch_fdt_eeprom(void *blob)
{
    struct Eeprom_list *tmp;

    list_for_each_entry(tmp, &eeprom_list.list, list){
        int article  = cpu_to_fdt32(tmp->eeprom.article);
        int revision = cpu_to_fdt32(tmp->eeprom.revision);
        do_fixup_by_path(blob,tmp->eeprom.name,"article",&article,sizeof(article),1);
        do_fixup_by_path(blob,tmp->eeprom.name,"rev",&revision,sizeof(revision),1);
    }

}

/*
 * Prepares a list of HWrevstruct structures \
 *     later used to update the fdt blob
 * */

static int do_update_fdt_eeprom(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    if (argc != 5)
        return CMD_RET_USAGE;

    struct Eeprom_list * tmp = (struct Eeprom_list*) malloc(sizeof(struct Eeprom_list));
    int ret=0;

     /* I2C chip address */
    tmp->eeprom.i2c_address  = simple_strtoul(argv[2], NULL, 16);

     /* I2C data address within the chip. */
    tmp->eeprom.i2c_offset = simple_strtoul(argv[3], NULL, 16);

     /* I2C bus number */
    tmp->eeprom.i2c_bus = simple_strtoul(argv[4], NULL, 16);

     /* Device tree node name */
    sprintf(tmp->eeprom.name,"/boards/%s",argv[1]);

    ret = eeprom_read_rev(&tmp->eeprom);

    if(ret==0)
    {
        list_add(&tmp->list,&eeprom_list.list);
        return CMD_RET_SUCCESS;
    }
    printf("Failed to read %s version \n",tmp->eeprom.name);

    cfree(tmp);
    return CMD_RET_FAILURE;
}

U_BOOT_CMD(
	update_fdt_eeprom, CONFIG_SYS_MAXARGS, 0, do_update_fdt_eeprom,
	"update_fdt_eeprom  <name> <address> <offset> <i2c>  ",
    "update_fdt_eeprom  <name> <address> <offset> <i2c>  \n"
	"update_fdt_eeprom  mainboard 0xae 0x40 3 will create an fdt node:\n"
" /boards{      \n "
"           mainboard {\n"
"           article=198752\n"
"           rev =1\n"
"           }\n"
);
