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
#include <fuse.h>
#include <asm/errno.h>
#include <i2c.h>

#define MAIN_EEPROM_I2C_ADDR 0xae


int GetMainboardVersion(int *article, int* revision)
{
    int ret = 1;
    int addr = 0x40;
    int chip = MAIN_EEPROM_I2C_ADDR >>1;
    struct
    {
        char article[10];
        char serial[10];
        char revision[4];
        char moduleOffset[2];
        char moduleDevice[2];
        char reserved[2];
        unsigned short checksum;
    } HWrev;    /**< 32 bytes including checksum */

    static int savedArticle;
    static int savedRevision;
#ifdef CONFIG_SYS_I2C
    if (!savedArticle)
    {

        ret = i2c_set_bus_num(2);
        ret = i2c_read(chip, addr, 1, (uchar*)&HWrev, sizeof(HWrev));
        if (ret == 0)
        {
            savedArticle  = simple_strtoul((char*)&HWrev.article[1], NULL, 10);
            savedRevision = simple_strtoul(HWrev.revision, NULL, 10);
           // printf("Mainboard article %d revision %d\n", savedArticle, savedRevision);
        }
        else
        {
            //printf("Failed reading article (%d)\n", ret);
        }
    }
    *article = savedArticle;
    *revision = savedRevision;
#endif
    return (ret > 0);
}


static int do_readmainboardarticle(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
    int article;
    int revision;
   GetMainboardVersion(&article, &revision);
   printf("Mainboard article %i\n ",article);
   char str[20];
   snprintf(str, strlen(str), "%i", article);
   setenv("mainboardarticle", str);
   return 0;
}

static int do_readmainboardrevision(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
    int article;
    int revision;
   GetMainboardVersion(&article, &revision);
   printf("Mainboard revision %i\n ",revision);
   char str[20];
   snprintf(str, strlen(str), "%i", revision);
   setenv("mainboardrevision", str);
   return 0;
}

U_BOOT_CMD(
	readmainboardrevision, CONFIG_SYS_MAXARGS, 0, do_readmainboardrevision,
	"Read the revision of the ec101 mainboard",
	"Read the revision of the ec101 mainboard\n"
);

U_BOOT_CMD(
	readmainboardarticle, CONFIG_SYS_MAXARGS, 0, do_readmainboardarticle,
	"Read the article of the ec101 mainboard",
	"Read the article of the ec101 mainboard\n"
);

