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
	BQ_SET_HIBERNATE,
	BQ_CLEAR_HIBERNATE,
	BQ_TEMPERATURE,
	BQ_VOLTAGE,
	BQ_STATE_OF_CHARGE,
	BQ_DESIGN_CAPACITY,
	BQ_RESET
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

/**
 * bq27_unseal - Transition to unsealed state
 *
 * Return: 0 on success
 */
int bq27_unseal(void);

/**
 * bq27_is_sealed - Query flash sealed status
 * @seal_status: 1 = sealed, 0 = unsealed, undef on fail
 *
 * Return: 0 on success
 */
int bq27_is_sealed(int *seal_status);

/**
 * bq27_init_complete - Check FW init status
 * @ready: 1 = init complete, 0 = init not complete
 *
 * Return: 0 on success
 */
int bq27_init_complete(int *ready);

/**
 * bq27_manufacturer_info - Read manufacturer info
 * @maninfo: Returned data string
 *
 * Return: 0 on success
 */
int bq27_manufacturer_info(char **maninfo);

#endif // _BQ27XXX_H

