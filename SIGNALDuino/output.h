// output.h
#ifndef _OUTPUT_h
#define _OUTPUT_h

#if !defined(MAPLE_Mini) && !defined(ESP32) && !defined(ARDUINO_ARCH_RP2040)
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#endif

#if !defined(MAPLE_Mini) && !defined(ESP32) && !defined(ARDUINO_ARCH_RP2040)
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))
#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))
#elif defined(MAPLE_Mini)
	#define pinAsInput(pin) pinMode(pin, INPUT)
	#define pinAsOutput(pin) pinMode(pin, OUTPUT)
	#define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)
	#define digitalLow(pin) digitalWrite(pin, LOW)
	#define digitalHigh(pin) digitalWrite(pin, HIGH)
	#define isHigh(pin) (digitalRead(pin) == HIGH)
	#define isLow(pin) (digitalRead(pin) == LOW)
#else
// ESP32 or ARDUINO_ARCH_RP2040
	#define pinAsInput(pin) pinMode(pin, INPUT)
	#define pinAsOutput(pin) pinMode(pin, OUTPUT)
	#define pinAsInputPullUp(pin) pinMode(pin, INPUT_PULLUP)

	#ifndef digitalLow
		#define digitalLow(pin) digitalWrite(pin, LOW)
	#endif
	#ifndef digitalHigh
		#define digitalHigh(pin) digitalWrite(pin, HIGH)
	#endif
	#ifndef isHigh
		#define isHigh(pin) (digitalRead(pin) == HIGH)
	#endif
	#ifndef isLow
		#define isLow(pin) (digitalRead(pin) == LOW)
	#endif
#endif

//#define DEBUG

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#endif

#ifdef ETHERNET_PRINT
#include <wificlient.h>

extern WiFiClient serverClient;


#define MSG_PRINTER serverClient // Not Implemented at this time
#else
#define MSG_PRINTER Serial
#endif

#ifdef ETHERNET_DEBUG
#define DBG_PRINTER Client // Not Implemented at this time
#else
#define DBG_PRINTER Serial
#endif


#define MSG_PRINT(...) { MSG_PRINTER.print(__VA_ARGS__); }
#define MSG_PRINTLN(...) { MSG_PRINTER.println(__VA_ARGS__); }
#define MSG_WRITE(...) { MSG_PRINTER.write(__VA_ARGS__); }

#ifdef DEBUG
	#define DBG_PRINT(...) {  DBG_PRINTER.print(__VA_ARGS__); }
	#define DBG_PRINTLN(...) { DBG_PRINTER.println(__VA_ARGS__); }
#else
	#define DBG_PRINT(...) 
	#define DBG_PRINTLN(...) 
#endif



#endif
