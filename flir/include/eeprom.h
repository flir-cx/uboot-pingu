

#ifndef __FLIR_EEPROM_H
#define __FLIR_EEPROM_H

struct Eeprom
{
	u8 i2c_bus;
	u16 i2c_address;
	u8 i2c_offset;
	u32 article;
	u32 revison;
	char name[32];
};

int get_eeprom_hwrev(struct Eeprom * eeprom);

#endif
