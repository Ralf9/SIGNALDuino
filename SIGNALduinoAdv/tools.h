// tools.h

#ifndef _tools_h
#define _tools_h

#ifndef MAPLE_Mini
#include <EEPROM.h>
//#elif defined(MAPLE_Mini) and ARDUINO > 190
//#include <EEPROM.h>
#endif

#include <Arduino.h>

extern uint16_t bankOffset;
extern String cmdstring;

namespace tools {
	
	void EEwrite(uint16_t, uint8_t);
	void EEstore();

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
	
	bool cmdstringPos2ip(uint8_t* ip, uint8_t pos, uint8_t EE_pos)	// eine ip im cmdstring im EEPROM merken und als array zurueckgeben
	{
		int8_t end;
		uint8_t i;
		uint32_t nr;
		uint8_t tmpIp[4];
		
		for (i = 0; i < 4; i++) {
			end = cmdstring.indexOf('.',pos);		// nach dem naechsten Punkt suchen
			if (end > 0) {
				nr = cmdstring.substring(pos, end).toInt();
			}
			else {
				nr = cmdstring.substring(pos).toInt();
			}
			if (nr < 256) {
				tmpIp[i] = (uint8_t)nr;
				pos = end + 1;
			}
			else {
				i = 4;
				break;
			}
			if (end <= 0) {	// letzter Wert
				break;
			}
		}
		if (i != 3) {
			return false;	// keine 4 Werte
		}
		
		for (i = 0; i < 4; i++) {
			//MSG_PRINTLN(tmpIp[i]);
			ip[i] = tmpIp[i];
			EEwrite(EE_pos+i,tmpIp[i]);
		}
		EEstore();
		return true;
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
	#else
		EEPROM.commit();
	#endif
	}
	
	inline void EEbufferFill(void)
	{
	#ifdef MAPLE_Mini
		eeprom_buffer_fill();
	#elif defined(ESP32)
		EEPROM.begin(900);
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
	
#ifdef MAPLE_Mini
	/*
	 * The width of the CRC calculation and result.
	 * Modify the typedef for a 16 or 32-bit CRC standard.
	 */
	typedef uint32_t crc;
	#define CRC_POLYNOMIAL 0x8005
	#define CRC_WIDTH  (8 * sizeof(crc))
	#define CRC_TOPBIT (1 << (CRC_WIDTH - 1))
	
	crc CRCs(unsigned char* message, uint16_t count)
	{
		crc remainder = 0;
		uint16_t byte;
		unsigned char bit;
	
		/*
		* Perform modulo-2 division, a byte at a time.
		 */
		for (byte = 0; byte < count; ++byte)
		{
			/*
			* Bring the next byte into the remainder.
			*/
			remainder ^= (message[byte] << (CRC_WIDTH - 8));
	
			/*
			* Perform modulo-2 division, a bit at a time.
			*/
			for (bit = 8; bit > 0; --bit)
			{
				/*
				 * Try to divide the current data bit.
				*/
				if (remainder & CRC_TOPBIT)
				{
					remainder = (remainder << 1) ^ CRC_POLYNOMIAL;
				}
				else
				{
					remainder = (remainder << 1);
				}
			}
		}
		
		/*
		* The final remainder is the CRC result.
		*/
		return (remainder);
	
	}   /* crcSlow() */


	#define STM32_UUID ((uint32_t *)UID_BASE)

	uint32_t flash_serial(void)	// Seriennummer des STM32 (UID) auslesen
	{
		uint32_t v[3];

		v[0] = STM32_UUID[0];
		v[1] = STM32_UUID[1];
		v[2] = STM32_UUID[2];

		return CRCs((unsigned char*)v,9);
	}
#endif
}
#endif
