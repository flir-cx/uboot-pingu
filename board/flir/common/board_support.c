// SPDX-License-Identifier: GPL-2.0+
#include "board_support.h"
#include "flir_generic.h"
#include "eeprom.h"

int board_support_setup(struct board_info *ioboard, struct hw_support *hardware)
{
	// Note: "evio" means the IO board, even if the name is
	// really specific to Evander
	int ret = eeprom_read_rev("evio", ioboard);

	if (ret)
		return ret;

	switch (ioboard->article) {
	case EVIO_ARTICLE:
	case EVIO2_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = true;
		strncpy(hardware->name, "Evander Camera", 20);
		break;

	case LEIF_ARTICLE:
	case LEIF2_ARTICLE:
	case LEIF4_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = true;
		hardware->usb_charge = true;
		strncpy(hardware->name, "Lennox Camera", 20);
		break;

	case DUPLO_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = false;
		strncpy(hardware->name, "Duplo Camera", 20);
		break;

	case SVIO_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = false;
		strncpy(hardware->name, "Svala Camera", 20);
		break;

	default:
		printf("Unknown camera, using default hw support %d\n",
		       ioboard->article);
	}

	return ret;
}
