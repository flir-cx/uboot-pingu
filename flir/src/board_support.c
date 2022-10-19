#include "../../../flir/include/board_support.h"
#include "../../../flir/include/flir_generic.h"
#include "../../../flir/include/eeprom.h"

int board_support_setup(struct eeprom *ioboard, struct hw_support *hardware)
{
	int ret = eeprom_read_rev(ioboard);
	if(ret != 0)
	{
		printf("Error reading io board %d \n",ret);
		return ret;
	}

	
	switch (ioboard->article_number) {
	case EVIO_ARTICLE:
	case EVIO2_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = true;
		strncpy(hardware->name, "Evander Camera",20);
		break;
		
	case LEIF_ARTICLE:
	case LEIF2_ARTICLE:
	case LEIF4_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = true;
		hardware->usb_charge = true;
		strncpy(hardware->name, "Lennox Camera",20);
		break;

	case DUPLO_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = false;
		strncpy(hardware->name, "Duplo Camera",20);
		break;

	case SVIO_ARTICLE:
		hardware->display = true;
		hardware->mipi_mux = false;
		hardware->usb_charge = false;
		strncpy(hardware->name, "Svala Camera",20);
		break;

	default:
		printf("Unknown camera, using default hw support %d \n",
		       ioboard->article_number);
	}
	
	return ret;
}
