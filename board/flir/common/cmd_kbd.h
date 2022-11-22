/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015 FLIR Systems.
 * Copyright (C) 2017 FLIR Systems.
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

#ifndef __CMD_KBD_H
#define __CMD_KBD_H

/**
 * read_keys(char **buf) - Read keyboard values
 * @buf: result ptr to null-terminated key buffer
 * Return: No of keys pressed, or -1 on error
 */
int read_keys(char **buf);

#define RECOVERY_KEY  "R"
#define SW_LOAD       "S"

#define cond_log_return(_cond, _ret, _fmt, args...)	\
({							\
	int _rval = (_ret);				\
	if (_cond) {					\
		printf(_fmt, ##args);			\
		return _rval;				\
	}						\
})

#define return_on_status(_ret, _fmt...) cond_log_return(_ret, _ret, ##_fmt)

#endif
