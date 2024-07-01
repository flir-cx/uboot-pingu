/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interface for communicating with the TI BQ27xxx
 * line of fuelgauges.
 *
 * Copyright (C) 2024 FLIR Systems.
 */
#ifndef _BQ27XXX_H
#define _BQ27XXX_H

/*
 * Command IDs correspond to BQ27 "Standard Commands",
 * see sluua35.pdf and sluub65b.pdf
 */
enum bq27_command {
	BQ_CONTROL_STATUS,
	BQ_DEVICE_TYPE,
	BQ_FW_VERSION,
	BQ_DF_VERSION,
	BQ_CHEM_ID,
	BQ_VOLTAGE,
	BQ_STATE_OF_CHARGE,
	BQ_DESIGN_CAPACITY
};

#define BQ27_DEFAULT_DESIGN_CAPACITY (1000)

/**
 * bq27_read_cmd - Read a device command
 * @cmd: Command ID
 * @rval: Return value storage
 *
 * Return: 0 on success
 */
int bq27_read_cmd(const enum bq27_command cmd, u16 *rval);

#endif // _BQ27XXX_H

