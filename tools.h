// tools.h

#ifndef _tools_h
#define _tools_h

#ifndef MAPLE_Mini
#include <EEPROM.h>
#endif
//#include "Storage.h"

extern uint16_t bankOffset;
extern String cmdstring;

namespace tools {

	uint8_t hex2int(uint8_t hex) {    // convert a hexdigit to int    // Todo: printf oder scanf nutzen
		if (hex >= '0' && hex <= '9') hex = hex - '0';
		else if (hex >= 'a' && hex <= 'f') hex = hex - 'a' + 10;
		else if (hex >= 'A' && hex <= 'F') hex = hex - 'A' + 10;
		return hex;
		// printf ("%d\n",$hex) ??
	}
	
	uint8_t cmdstringPos2int(uint8_t pos) {
		uint8_t val;
		uint8_t hex;

		hex = (uint8_t)cmdstring.charAt(pos);
		val = hex2int(hex) * 16;
		hex = (uint8_t)cmdstring.charAt(pos+1);
		val = hex2int(hex) + val;
		return val;
	}

	void EEwrite(uint16_t adr, uint8_t val)
	{
		//eeprom.setByte(adr, val);
	#ifdef MAPLE_Mini
		eeprom_buffered_write_byte(adr, val);
	#else
		EEPROM.write(adr, val);
	#endif
	}
	
	void EEbankWrite(uint8_t reg, uint8_t val)
	{
		//eeprom.setByte(bankOffset + reg, val);
	#ifdef MAPLE_Mini
		eeprom_buffered_write_byte(bankOffset + reg, val);
	#else
		EEPROM.write(bankOffset + reg, val);
	#endif
	}
	
	void EEstore(void)
	{
		//eeprom.store();
	#ifdef MAPLE_Mini
		eeprom_buffer_flush();  // Copy the data from the buffer to the flash
	#endif
	}
	
	inline void EEbufferFill(void)
	{
	#ifdef MAPLE_Mini
		eeprom_buffer_fill();
	#endif
	}
	
	uint8_t EEread(uint16_t adr)
	{
		//return eeprom.getByte(adr);
		#ifdef MAPLE_Mini
			return eeprom_buffered_read_byte(adr);
		#else
			return EEPROM.read(adr);
		#endif
	}

	uint8_t EEbankRead(uint8_t reg)
	{
		return EEread(bankOffset + reg);
	}
}
#endif
