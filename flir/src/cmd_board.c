#include <common.h>
#include <command.h>
#include <asm/errno.h>
#include <malloc.h>
#include "eeprom.h"


#define Eeprom_entry(bus,address,i2c_offset,name) {bus,address,i2c_offset,0,0,name}

struct Eeprom eeprom [] =
{
	Eeprom_entry(2,0xae,0x40,"main"),
	Eeprom_entry(2,0xae,0x40,"ec101"),
	Eeprom_entry(2,0xaa,0x00,"evio"),
};
static int do_board(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
    if (argc != 2)
        return CMD_RET_USAGE;
	int i,ret;

	for(i=0;i<ARRAY_SIZE(eeprom);i++)
		if(!strcmp(eeprom[i].name,argv[1]))
			break;

	if(i>=ARRAY_SIZE(eeprom))
	{
		printf("Board %s unsupported\n",argv[1]);
		return CMD_RET_FAILURE;
	}

	ret = get_eeprom_hwrev(&eeprom[i]);
	if(ret == 0)
	{
		char env[32];
		char var[20];
		snprintf(env, strlen(env), "%s_board_revision", eeprom[i].name);
		snprintf(var, strlen(var), "%i", eeprom[i].revison);
	    setenv(env, var);

		snprintf(env, strlen(env), "%s_board_article", eeprom[i].name);
		snprintf(var, strlen(var), "%i", eeprom[i].article);
	    setenv(env, var);
		return CMD_RET_SUCCESS;
	}
	printf("Board %s not responding \n",eeprom[i].name);
    return CMD_RET_FAILURE;
}

U_BOOT_CMD(
	board, CONFIG_SYS_MAXARGS, 0, do_board,
	"read rev and article from eeprom   ",
	"<name>"
	"\nWill read from eeprom <name> and fill in env variable\n"
	"	<name>_board_revision and \n"
	"	<name>_board_article\n"
	"supported eeproms: \n"
		"main, ec101, evio"
);
