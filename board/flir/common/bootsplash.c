// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 FLIR Systems.
 */

#include <env.h>
#include <command.h>
#include <linux/errno.h>
#include "../common/usbcharge.h"

/**
 * @brief Overrides the (weak) splash_screen_prepare in splash.c
 *
 * @return int non zero on error
 */
int splash_screen_prepare(void)
{
	char *env_loadsplash;

	set_boot_logo();

	if (!env_get("splashimage")) {
		log_err("Environment variable splashimage not found!\n");
		return -EINVAL;
	}

	env_loadsplash = env_get("loadsplash");
	if (!env_loadsplash) {
		log_err("Environment variable loadsplash not found!\n");
		return -EINVAL;
	}

	if (run_command_list(env_loadsplash, -1, 0)) {
		log_err("Failed to run loadsplash %s\n\n", env_loadsplash);
		return -ENOENT;
	}
	return 0;
}
