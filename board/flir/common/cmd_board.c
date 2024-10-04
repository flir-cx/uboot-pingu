#include <common.h>
#include <command.h>
#include <errno.h>
#include "eeprom.h"

#define SUPP_BRD_STR "main, ec101, ec201, ec202, ec302, ec401w, ec501, eoco, evio"

static int do_board(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char env[64];
	char var[32];
	struct hw_version board;

	if (argc != 2)
		return CMD_RET_USAGE;

	if (eeprom_read_rev(argv[1], &board))
		return 1;

	snprintf(env, sizeof(env), "%s_board_revision", argv[1]);
	snprintf(var, sizeof(var), "%i", board.revision);
	env_set(env, var);

	snprintf(env, sizeof(env), "%s_board_article", argv[1]);
	snprintf(var, sizeof(var), "%i", board.article);
	env_set(env, var);

	return (board.article == 0);
}

#ifdef CONFIG_FLIR_OLD_COMMAND_STYLE
U_BOOT_CMD(board, CONFIG_SYS_MAXARGS, 0, do_board,
	   "read rev and article from eeprom   ",
	   "<name>"
	   "\nWill read from eeprom <name> and fill in env variable\n"
	   "	<name>_board_revision and\n"
	   "	<name>_board_article\n"
	   "supported eeproms:\n"
	   SUPP_BRD_STR
	   "\n\nReturns fail when article is unknown.\n"
	   "Note: 'evio' usually means 'any io board'\n"
	   );
#endif

U_BOOT_CMD(flir_board, CONFIG_SYS_MAXARGS, 0, do_board,
	   "read rev and article from eeprom   ",
	   "<name>"
	   "\nWill read from eeprom <name> and fill in env variable\n"
	   "	<name>_board_revision and\n"
	   "	<name>_board_article\n"
	   "supported eeproms:\n"
	   SUPP_BRD_STR
	   "\n\nReturns fail when article is unknown.\n"
	   "Note: 'evio' usually means 'any io board'\n"
);
