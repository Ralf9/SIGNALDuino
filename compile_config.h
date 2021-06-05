#pragma once

// Config flags for compiling correct options / boards Define only one

//#define MAPLE_SDUINO 1
//#define MAPLE_CUL 1
//#define BLACK_BOARD 1  // 1 - USB, 2 - serial USART2 fuer ESP
#define SIGNALESP32 1
//#define EVIL_CROW_RF 1
//#define ESP32_SDUINO_TEST 1

//#define LAN_WIZ 1        // nur fuer MAPLE_SDUINO mit USR-ES1 W5500
#define LAN_INIT_DHCP 1  // damit wird bei der ersten Inbetriebnahme DHCP verwendet
//#define MAPLE_WATCHDOG 1

//#define DEBUG_BackupReg 1
//#define DEBUG_SERIAL 2	// debug level
#define SerialNr USART1	// serial2 = USART2

//#define DEBUGSENDCMD  1

// ---- this options are current not possible, because the V 4.x firmware is only for STM32 (Maple Mini)
//#define ARDUINO_ATMEGA328P_MINICUL 1
//#define OTHER_BOARD_WITH_CC1101  1

//#define WATCHDOG	1 // Der Watchdog ist in der Entwicklungs und Testphase deaktiviert. Es muss auch ohne Watchdog stabil funktionieren.

// ---------------------------------------
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
#ifdef BLACK_BOARD
	#define MAPLE_Mini
	#define CMP_CC1101
#endif
#if BLACK_BOARD == 2
	#define SERIAL_USART2
#endif
#ifdef SIGNALESP32
	#define CMP_CC1101
#endif
#ifdef ESP32_SDUINO
	#define CMP_CC1101
#endif
#ifdef EVIL_CROW_RF
	#define CMP_CC1101
#endif
#ifdef ESP32_SDUINO_TEST
	#define CMP_CC1101
#endif
