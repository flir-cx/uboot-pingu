// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 FLIR Systems.
 */
#include <command.h>
#include "bq27xxx.h"

int do_bq27(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	u16 value = 0;

	if (argc < 2) {
		log_info("Too few arguments\n");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		ret = bq27_read_cmd(BQ_CONTROL_STATUS, &value);
	} else if (!strcmp(argv[1], "type")) {
		ret = bq27_read_cmd(BQ_DEVICE_TYPE, &value);
	} else if (!strcmp(argv[1], "fwver")) {
		ret = bq27_read_cmd(BQ_FW_VERSION, &value);
	} else if (!strcmp(argv[1], "dfver")) {
		ret = bq27_read_cmd(BQ_DF_VERSION, &value);
	} else if (!strcmp(argv[1], "chemid")) {
		ret = bq27_read_cmd(BQ_CHEM_ID, &value);
	} else if (!strcmp(argv[1], "volt")) {
		ret = bq27_read_cmd(BQ_VOLTAGE, &value);
	} else if (!strcmp(argv[1], "soc")) {
		ret = bq27_read_cmd(BQ_STATE_OF_CHARGE, &value);
	} else if (!strcmp(argv[1], "dcap")) {
		ret = bq27_read_cmd(BQ_DESIGN_CAPACITY, &value);
	} else {
		log_err("Invalid command '%s'\n", argv[1]);
		ret = 1;
	}

	if (!ret)
		log_info("Response: 0x%04x (%u)\n", value, value);
	else
		log_info("Command failed\n");

	return 0;
}

U_BOOT_CMD(bq27, 3, 0, do_bq27,
	   "Read information from a bq27xxx battery fuelgauge",
	   "<command>\n\n"
	   "bq27 status  - Control(CONTROL_STATUS)\n"
	   "bq27 type    - Control(DEVICE_TYPE)\n"
	   "bq27 fwver   - Control(FW_VERSION)\n"
	   "bq27 dfver   - Control(DF_VERSION)\n"
	   "bq27 chemid  - Control(CHEM_ID)\n"
	   "bq27 volt    - Voltage()\n"
	   "bq27 soc     - StateOfCharge()\n"
	   "bq27 dcap    - DesignCapacity()\n"
	);
