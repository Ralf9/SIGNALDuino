#pragma once

// Config flags for compiling correct options / boards Define only one

#define ARDUINO_ATMEGA328P_MINICUL 1
//#define ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101 1
//#define ARDUINO_BUSWARE_CUL 1                                 // BusWare CUL V3 (ATmega32U4)
//#define OTHER_BOARD_WITH_CC1101  1


//#define DEBUGSENDCMD  1
//#define SENDTODECODER 1 // damit wird in der send_raw Routine anstatt zu senden, die Pulse direkt dem Decoder uebergeben

//#define WATCHDOG	1 // Der Watchdog ist in der Entwicklungs und Testphase deaktiviert. Es muss auch ohne Watchdog stabil funktionieren.
//#define CMP_MEMDBG 1

// ---------------------------------------
// Do not Change anything below this line

#ifdef OTHER_BOARD_WITH_CC1101
	#define CMP_CC1101
#endif
#ifdef ARDUINO_ATMEGA328P_MINICUL
	#define CMP_CC1101
#endif
#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	#define CMP_CC1101
	#define ONLY_FSK
#endif
#ifdef ARDUINO_BUSWARE_CUL
	#define CMP_CC1101
	#define ONLY_FSK
#endif
