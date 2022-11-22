#ifndef __FLIR_GENERIC_H
#define __FLIR_GENERIC_H

#include <common.h>

#define EVIO_ARTICLE 199159
#define EVIO2_ARTICLE 300504
#define LEIF_ARTICLE 199287
#define LEIF2_ARTICLE 199581
#define LEIF4_ARTICLE 300503
#define DUPLO_ARTICLE 0
#define SVIO_ARTICLE 199489

struct hw_support
{
	bool mipi_mux;   //display is connected through a mipi mux
	bool display;    //camera have a display
	bool usb_charge; //supports usb chargeing
	char name [20];
};

struct epit {
	u32 cr;
	u32 sr;
	u32 lr;
	u32 cmpr;
	u32 cnr;
};

#endif  /* __FLIR_GENERIC_H */
