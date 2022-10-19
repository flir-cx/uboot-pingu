#include <common.h>
#include <command.h>
#include <errno.h>
#include <malloc.h>
#include "../../../flir/include/eeprom.h"

#define Eeprom_entry(bus, addr, offs, nam) {		\
		.i2c_bus = bus,			\
		.i2c_address = addr,		\
		.i2c_offset = offs,		\
		.article_number = 0,		\
		.article_revision = 0,		\
		.name = nam }

struct eeprom eeprom [] =
{
	Eeprom_entry(2, 0xae, 0x40, "main"),
	Eeprom_entry(2, 0xae, 0x40, "ec101"),
	Eeprom_entry(0, 0xae, 0x40, "eoco"),
	Eeprom_entry(2, 0xaa, 0x00, "evio"),
	Eeprom_entry(6, 0xae, 0x40, "ec401w"),
};

static int do_board(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc != 2)
		return CMD_RET_USAGE;
	int i, ret;

	for(i = 0; i < ARRAY_SIZE(eeprom); i++)
		if (!strcmp(eeprom[i].name, argv[1]))
			break;

	if(i >= ARRAY_SIZE(eeprom))
	{
		printf("Board %s unsupported\n", argv[1]);
		return CMD_RET_FAILURE;
	}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
	ret = eeprom_read_rev(&eeprom[i]);
	if (ret == 0)
	{
		char env[64];
		char var[32];
		snprintf(env, sizeof(env), "%s_board_revision", eeprom[i].name);
		snprintf(var, sizeof(var), "%i", eeprom[i].article_revision);
		env_set(env, var);

		snprintf(env, sizeof(env), "%s_board_article", eeprom[i].name);
		snprintf(var, sizeof(var), "%i", eeprom[i].article_number);
		env_set(env, var);
		return CMD_RET_SUCCESS;
	} else {
		printf("Board %s not responding \n", eeprom[i].name);
		return CMD_RET_FAILURE;
	}
#endif

	printf("Board %s not responding \n", eeprom[i].name);
	return CMD_RET_SUCCESS;
	return CMD_RET_FAILURE;
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(
	   board, CONFIG_SYS_MAXARGS, 0, do_board,
	   "read rev and article from eeprom   ",
	   "<name>"
	   "\nWill read from eeprom <name> and fill in env variable\n"
	   "	<name>_board_revision and \n"
	   "	<name>_board_article\n"
	   "supported eeproms: \n"
	   "ec401w ec101"
	   );
#endif

U_BOOT_CMD(
	   flir_board, CONFIG_SYS_MAXARGS, 0, do_board,
	   "read rev and article from eeprom   ",
	   "<name>"
	   "\nWill read from eeprom <name> and fill in env variable\n"
	   "	<name>_board_revision and \n"
	   "	<name>_board_article\n"
	   "supported eeproms: \n"
	   "main, ec101, eoco, evio"
);
