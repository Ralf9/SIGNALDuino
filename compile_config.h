#pragma once

// Config flags for compiling correct options / boards Define only one

#define MAPLE_SDUINO 1
//#define MAPLE_CUL 1
//#define LAN_WIZ 1
#define MAPLE_WATCHDOG 1

//#define DEBUG_BackupReg 1
//#define DEBUG_SERIAL 2	// debug level
#define SerialNr USART1	// serial2 = USART2

//#define ARDUINO_ATMEGA328P_MINICUL 1
//#define OTHER_BOARD_WITH_CC1101  1
//#define CMP_MEMDBG 1

//#define DEBUGSENDCMD  1

// Do not Change anything below this line

#ifdef OTHER_BOARD_WITH_CC1101
	#define CMP_CC1101
#endif
#ifdef ARDUINO_ATMEGA328P_MINICUL
	#define CMP_CC1101
#endif
#ifdef MAPLE_SDUINO
	#define MAPLE_Mini
	#define CMP_CC1101
#endif
#ifdef MAPLE_CUL
	#define MAPLE_Mini
	#define CMP_CC1101
#endif
