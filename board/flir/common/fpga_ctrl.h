/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2023 FLIR Systems.
 */
#ifndef __FPGA_CTRL_H
#define __FPGA_CTRL_H

struct fpga_pins {
	int init_n;
	int program_n;
	int config_n;
	int status_n;
	int ce;

	int done;
};

struct fpga_ctrl;

/**
 * @brief Operations that may be implemented in the <fpga_name>_fpga_ctrl.c
 *
 * If not implemented the driver will return 0 or ok apart from
 * fpga_poll_config_status which will return -1 to make sure that we
 * get stuck with an error if not implemented.
 */
struct fpga_ops {
	int (*fpga_request_pinctl)(struct fpga_ctrl *fpga);
	int (*fpga_release_pinctl)(struct fpga_ctrl *fpga);

	int (*fpga_enable)(struct fpga_ctrl *fpga);
	int (*fpga_disable)(struct fpga_ctrl *fpga);

	int (*fpga_hold_in_config)(struct fpga_ctrl *fpga);
	int (*fpga_start_config)(struct fpga_ctrl *fpga);

	int (*fpga_poll_config_status)(struct fpga_ctrl *fpga);
};

/**
 * @brief These are implemented in the board file
 * As it is implemented now fpga_enable_power is supposed to enable
 * power to the shared spi_flash as well as to the fpga.
 *
 */
struct fpga_board_ops {
	int (*fpga_request_flash_spi)(struct fpga_ctrl *fpga);
	int (*fpga_release_flash_spi)(struct fpga_ctrl *fpga);
	int (*fpga_init_spi_flash)(struct fpga_ctrl *fpga);
	int (*fpga_enable_power)(struct fpga_ctrl *fpga);
};

/**
 * @brief Struct that contains what is needed for handling the fpga
 *
 */
struct fpga_ctrl {
	struct fpga_pins pins;
	struct fpga_ops ops;
	struct fpga_board_ops board_ops;
};

/**
 * fpga_init_ctrl() - Implement this in the board file where fpga and pins are known.
 * For example usage see mx6ec101.c:
 *
 * @param fpga Struct allocated from caller that will be filled in
 *
 */
void fpga_init_ctrl(struct fpga_ctrl *fpga);

/**
 * @brief Put fpga struct into a known initial state.
 *	  Resets all pins to -1 and structs to zero.
 *
 * @param fpga Struct allocated from caller that will be zeroed
 *
 * @return 0 if OK, -ve on error
 */
int fpga_zero_ctrl(struct fpga_ctrl *fpga);

/**
 * @brief Request control over gpio pins used to initiate
 *	  program sequence of the fpga. Typically program, init, done etc.
 *
 * @param fpga Completely initialized fpga struct
 *
 * @return 0 if OK, -ve on error
 */
int fpga_request_pinctl(struct fpga_ctrl *fpga);

/**
 * @brief Release control over gpio pins used to initiate
 *	  program sequence of the fpga. Typically program, init, done etc.
 *
 * @param fpga Completely initialized fpga struct
 *
 * @return 0 if OK, -ve on error
 */
int fpga_release_pinctl(struct fpga_ctrl *fpga);

/**
 * @brief Takes ownership of the spi interface used for the flash shared
 *	  with the fpga.
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_request_flash_spi(struct fpga_ctrl *fpga);

/**
 * @brief Releases ownership of the spi interface used for the flash shared
 *	  with the fpga. This is needed to handover ownership to the fpga
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_release_flash_spi(struct fpga_ctrl *fpga);

/**
 * @brief Implement this if you need to configure the flash before handing it
 *	  over to the fpga. E.g. disable hold or 3 byte addressing
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_init_spi_flash(struct fpga_ctrl *fpga);

/**
 * @brief Implement this if you need to have a chip enable to be triggered after
 *	  programming is complete
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_enable(struct fpga_ctrl *fpga);

/**
 * @brief Implement this if you need to have a something done before
 *	  restarting the programming of the fpga
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_disable(struct fpga_ctrl *fpga);

/**
 * @brief Implement this to set and hold the fpga in configuration mode
 *	  if it is not done by default.
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_hold_in_config(struct fpga_ctrl *fpga);

/**
 * @brief Implement this to trigger a start config sequence of the fpga
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_start_config(struct fpga_ctrl *fpga);

/**
 * @brief Implement this to power up the fpga
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if OK, -ve on error
 */
int fpga_enable_power(struct fpga_ctrl *fpga);

/**
 * @brief Implement this to poll config status
 *	  This is mandatory to implement for an fpga
 *
 * @param fpga Completely initialized fpga struct
 * @return 0 if not done, 1 if done, -ve on error that is not recoverable
 */
int fpga_poll_config_status(struct fpga_ctrl *fpga);

/**
 * @brief Set operations for a type of fpga. Implement in fpga_ctrl_<name>.c
 *
 * @param ctrl in which to fill ops
 */
void fpga_set_ops(struct fpga_ctrl *ctrl);

#endif /* __FPGA_CTRL_H */
