#include <common.h>
#include <console.h>
#include <mapmem.h>
#include <dm/test.h>
#include <test/ut.h>
#include <test/suites.h>
#include <test/test.h>

/* Declare a new test */
#define FLIR_TEST(_name, _flags)   UNIT_TEST(_name, _flags, flir_test)

/* Tests go here */
static int test_test(struct unit_test_state *uts)
{
	
	return 0;
}
FLIR_TEST(test_test, UT_TESTF_CONSOLE_REC);

/* At the bottom of the file: */
int do_ut_flir(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct unit_test *tests = ll_entry_start(struct unit_test, flir_test);
	const int n_ents = ll_entry_count(struct unit_test, flir_test);

	return cmd_ut_category("cmd_flir", "flir_test_", tests, n_ents, argc, argv);
}
