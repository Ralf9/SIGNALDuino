/*
*   RF_RECEIVER v4.xx for Arduino
*   Sketch to use an arduino as a receiver/sending device for digital signals
*
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented in the sketch,
*   there is an option to send almost any data over a send raw interface
*   2014-2015  N.Butzek, S.Butzek
*   2016 S.Butzek

*   This software focuses on remote sensors like weather sensors (temperature,
*   humidity Logilink, TCM, Oregon Scientific, ...), remote controlled power switches
*   (Intertechno, TCM, ARCtech, ...) which use encoder chips like PT2262 and
*   EV1527-type and manchester encoder to send information in the 433MHz Band.
*   But the sketch will also work for infrared or other medias. Even other frequencys
*   can be used
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.


	Version from: https://github.com/Ralf9/SIGNALDuino/blob/dev-r41x_cc110

*-------------------------------------------------------------------------------------------------------------------
	
*/


// Config flags for compiling correct options / boards Define only one

#define MAPLE_SDUINO 1
//#define MAPLE_CUL 1
//#define LAN_WIZ 1
//#define ARDUINO_ATMEGA328P_MINICUL 1
//#define OTHER_BOARD_WITH_CC1101  1
//#define CMP_MEMDBG 1

// *** bitte auch das "#define LAN_WIZ 1" in der SignalDecoder.h beachten ***
// *** bitte auch das "#define LAN_WIZ 1" in der USBD_reenumerate.c beachten ***

// bitte auch das "#define CMP_CC1101" in der SignalDecoder.h beachten

#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "4.1.1-dev200504"
#define VERSION_1               0x41
#define VERSION_2               0x0d

#ifdef OTHER_BOARD_WITH_CC1101
	#define CMP_CC1101
#endif
#ifdef ARDUINO_ATMEGA328P_MINICUL
	#define CMP_CC1101
#endif
#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
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

#ifdef CMP_CC1101
	#ifdef ARDUINO_ATMEGA328P_MINICUL  // 8Mhz 
		#define PIN_LED               4
		#define PIN_SEND              2   // gdo0Pin TX out
		#define PIN_RECEIVE           3
		#define PIN_MARK433	      A0
	#elif MAPLE_SDUINO
		#define PIN_LED              33
		#define PIN_SEND             17   // gdo0 Pin TX out
	    #define PIN_RECEIVE          18   // gdo2
		#define PIN_WIZ_RST          27
	#elif MAPLE_CUL
		#define PIN_LED              33
		#define PIN_SEND             17   // gdo0 Pin TX out
	    #define PIN_RECEIVE          18
		#define PIN_WIZ_RST          27
	#else 
		#define PIN_LED               9
		#define PIN_SEND              3   // gdo0Pin TX out
	    #define PIN_RECEIVE           2
	#endif
#else
	#define PIN_RECEIVE            2
	#define PIN_LED                13 // Message-LED
	#define PIN_SEND               11
#endif


#ifdef MAPLE_Mini
	#define BAUDRATE               115200
	#define FIFO_LENGTH            170
	const uint8_t pinReceive[] = {11, 18, 16, 14};
#else
	#define BAUDRATE               57600
	#define FIFO_LENGTH            140 // 50
#endif

//#define WATCHDOG	1 // Der Watchdog ist in der Entwicklungs und Testphase deaktiviert. Es muss auch ohne Watchdog stabil funktionieren.
//#define DEBUGSENDCMD  1
//(#define SENDTODECODER 1) ist neu ccmode=15 -> damit wird in der send_raw Routine anstatt zu senden, die Pulse direkt dem Decoder uebergeben

#define DEBUG                  1

#ifdef WATCHDOG
	#include <avr/wdt.h>
#endif

//---------------------------------------------------------------------------------------------

#include "cc1101.h"
#include "FastDelegate.h"
#include "output.h"
#include "bitstore.h"
#include "signalDecoder4.h"
#include "SimpleFIFO.h"

SimpleFIFO<int16_t,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;

#ifdef MAPLE_Mini
  #include <malloc.h>
  extern char _estack;
  extern char _Min_Stack_Size;
  static char *ramend = &_estack;
  static char *minSP = (char*)(ramend - &_Min_Stack_Size);
  extern "C" char *sbrk(int i);
#else
  #include <TimerOne.h>  // Timer for LED Blinking
#endif
  
#ifdef LAN_WIZ
  #include <SPI.h>
  #include <Ethernet.h>

  EthernetServer server = EthernetServer(23);
  EthernetClient client;
#endif

#define pulseMin  90

#ifdef MAPLE_Mini
  #define maxCmdString 600
#else
  #define maxCmdString 350
#endif

#define maxSendPattern 10
#define mcMinBitLenDef   17
#define ccMaxBuf 64
#define defMaxMsgSize 1500	// selber Wert wie in signalDecoder4.h
#define maxSendEcho 100

#define radioOokAsk 1
#define defSelRadio 1	// B
#define defStatRadio 0xFF
// EEProm Address
#define EE_MAGIC_OFFSET      0
#define addr_togglesec       0x3C
#define addr_ccN             0x3D
#define addr_ccmode          0x3E
//#define addr_featuresB       0x3F
#define addr_statRadio       0xEB    // A=EB B=EC C=ED D=EE  Bit 0-3 Bank,  1F-Init, Bit 6 = 1 - Fehler bei Erkennung, Bit 6&7 = 1 - Miso Timeout, FF-deaktiviert
#define addr_selRadio        0xEF
//#define addr_bank            0xFD
#define addr_features        0xFF
// Ethernet EEProm Address
#define EE_MAC_ADDR    0xC0
#define EE_IP4_ADDR    0xC8
#define EE_IP4_GATEWAY 0xCC
#define EE_IP4_NETMASK 0xD0

const uint8_t mac_def[] = { 0x00, 0x80, 0x41 };
const uint8_t ip_def[] = { 192, 168, 0, 244 };
const uint8_t gateway_def[] = { 192, 168, 0, 1 };
const uint8_t netmask_def[] = { 255, 255, 255, 0 };

volatile bool blinkLED = false;
String cmdstring = "";
char msg_cmd0 = ' ';
char msg_cmd1 = ' ';
volatile unsigned long lastTime = micros();
bool hasCC1101 = false;
bool LEDenabled = true;
bool toggleBankEnabled = false;
bool RXenabled[] = {false, false, false, false};	// true - enable receive, Zwischenspeicher zum enablereceive merken
bool unsuppCmd = false;
bool CmdOk = false;
uint8_t MdebFifoLimit = 120;
uint8_t bank = 0;
uint16_t bankOffset = 0;
//uint8_t ccN = 0;
uint8_t ccmode = 0;		// cc1101 Mode: 0 - normal, 1 - FIFO, 2 - FIFO ohne dup, 3 - FIFO LaCrosse, 9 - FIFO mit Debug Ausgaben
uint8_t radionr = defSelRadio;
uint8_t radio_bank[4];
uint8_t ccBuf[4][ccMaxBuf];
// Ethernet
uint8_t mac[6];
uint8_t ip[4];
uint8_t gateway[4];
uint8_t netmask[4];

void cmd_help_S();
void cmd_help();
void cmd_bank();
void configCMD();
void configRadio();
void getConfig();
void configSET();
void ccRegWrite();
void cmd_config();
void cmd_configFactoryReset();
void cmd_ccFactoryReset();
void getPing();
void cmd_readEEPROM();
void cmd_freeRam();
void cmd_send();
void cmd_uptime();
void cmd_toggleBank();
void cmd_Version();
void cmd_writeEEPROM();
void cmd_writePatable();
void changeReceiver();

//typedef void (* GenericFP)(int); //function pointer prototype to a function which takes an 'int' an returns 'void'
#define cmdAnz 23
const char cmd0[] =  {'?', '?', 'b', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'e', 'e', 'P', 'r', 'R', 'S', 't', 'T', 'V', 'W', 'x', 'X', 'X'};
const char cmd1[] =  {'S', ' ', ' ', 'E', 'D', 'G', 'R', 'S', 'W', ' ', 'C', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'E', 'Q'};
const bool cmdCC[] = {  0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,  0 };
void (*cmdFP[])(void) = {
		cmd_help_S, // ?S
		cmd_help,	// ?
		cmd_bank,	// b
		configCMD,	// CE
		configCMD,	// CD
		getConfig,	// CG
		configRadio,// CR
		configSET,	// CS
		ccRegWrite,	// CW
		cmd_config,	// C
		cmd_configFactoryReset,	// eC
		cmd_ccFactoryReset,		// e
		getPing,	// P
		cmd_readEEPROM,	// r
		cmd_freeRam,	// R
		cmd_send,	// S
		cmd_uptime,	// t
		cmd_toggleBank, // T
		cmd_Version,	// V
		cmd_writeEEPROM,// W
		cmd_writePatable,// x
		changeReceiver,	// XE
		changeReceiver	// XQ
		};

#define CSetAnz 10
#define CSetAnzEE 12
#define CSet16 8
#define CSccN 6
#define CSccmode 7

//const char *CSetCmd[] = {"fifolimit", "mcmbl", "mscnt", "maxMuPrintx256", "maxMsgSizex256", "maxnumpat", "ccN",    "ccmode", "muthresh", "L",  "maxpulse", "L"  };
const uint8_t CSetAddr[] = {  0xf0,     0xf1,     0xf2,          0xf3,           0xf4,            0xf5,   addr_ccN, addr_ccmode,     0xf8,  0xf9,  0xfa,     0xfb };
const uint8_t CSetDef[] =  {    120,       0,        4,             3,              4,               8,          0,           0,        0,     0,     0,        0 };

const char string_0[] PROGMEM = "fifolimit";
const char string_1[] PROGMEM = "mcmbl";
const char string_2[] PROGMEM = "mscnt";
const char string_3[] PROGMEM = "maxMuPrintx256";
const char string_4[] PROGMEM = "maxMsgSizex256";
const char string_5[] PROGMEM = "maxnumpat";
const char string_6[] PROGMEM = "ccN";
const char string_7[] PROGMEM = "ccmode";
const char string_8[] PROGMEM = "muthresh";
const char string_9[] PROGMEM = "maxpulse";

const char * const CSetCmd[] PROGMEM = { string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9};

#ifdef CMP_MEMDBG

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;
uint8_t *heapptr, *stackptr;
uint16_t diff=0;
void check_mem() {
 stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
 heapptr = stackptr;                     // save value of heap pointer
 free(stackptr);      // free up the memory again (sets stackptr to 0)
 stackptr =  (uint8_t *)(SP);           // save value of stack pointer
}
//extern int __bss_end;
//extern void *__brkval;

int get_free_memory()
{
 int free_memory;

 if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
 else
   free_memory = ((int)&free_memory) - ((int)__brkval);

 return free_memory;
}


int16_t ramSize=0;   // total amount of ram available for partitioning
int16_t dataSize=0;  // partition size for .data section
int16_t bssSize=0;   // partition size for .bss section
int16_t heapSize=0;  // partition size for current snapshot of the heap section
int16_t stackSize=0; // partition size for current snapshot of the stack section
int16_t freeMem1=0;  // available ram calculation #1
int16_t freeMem2=0;  // available ram calculation #2

#endif  // CMP_MEMDBG

void handleInterrupt();
void enableReceive();
void disableReceive();
void serialEvent();
void cronjob();
int freeRam();
void HandleCommand();
bool command_available=false;
unsigned long getUptime();
void storeFunctions(const int8_t ms=1, int8_t mu=1, int8_t mc=1, int8_t red=1, int8_t deb=0, int8_t led=1, int8_t overfl=0);
void getFunctions(bool *ms,bool *mu,bool *mc, bool *red, bool *deb, bool *led, bool *overfl);
void initEEPROM(void);
void setCCmode();
void print_Bank();
void print_radio_sum();
uint16_t getBankOffset(uint8_t tmpBank);
uint8_t radioDetekt(bool confmode, uint8_t Dstat);
void printHex2(const byte hex);
uint8_t rssiCallback() { return 0; };	// Dummy return if no rssi value can be retrieved from receiver


void setup() {
#ifdef MAPLE_Mini
	pinAsOutput(PIN_WIZ_RST);
#ifndef LAN_WIZ
	digitalWrite(PIN_WIZ_RST, HIGH);
#endif
#endif
tools::EEbufferFill();
getEthernetConfig();
#ifdef LAN_WIZ
	digitalWrite(PIN_WIZ_RST, LOW);		// RESET should be heldlowat least 500 us for W5500
	delayMicroseconds(500);
	digitalWrite(PIN_WIZ_RST, HIGH);
	Ethernet.begin(mac, ip, gateway, netmask);
	server.begin();		// start listening for clients
#else
	Serial.begin(BAUDRATE);
	//while (!Serial) {
	//	; // wait for serial port to connect. Needed for native USB
	//}
	for (uint8_t sw=0;sw<255;sw++ ) {
		delay(10);
		if (Serial) {
			break;
		}
	}
#endif
	if (musterDec.MdebEnabled) {
		DBG_PRINTLN(F("Using sFIFO"));
	}
#ifdef WATCHDOG
	if (MCUSR & (1 << WDRF)) {
		MSG_PRINTLN(F("Watchdog caused a reset"));
	}
	/*
	if (MCUSR & (1 << BORF)) {
		DBG_PRINTLN("brownout caused a reset");
	}
	if (MCUSR & (1 << EXTRF)) {
		DBG_PRINTLN("external reset occured");
	}
	if (MCUSR & (1 << PORF)) {
		DBG_PRINTLN("power on reset occured");
	}
	*/
	wdt_reset();

	wdt_enable(WDTO_2S);  	// Enable Watchdog
#endif
	//delay(2000);
	pinAsInput(PIN_RECEIVE);
	pinAsOutput(PIN_LED);
	// CC1101
#ifdef WATCHDOG
	wdt_reset();
#endif
#ifdef CMP_CC1101
	cc1101::setup();
#endif
  	initEEPROM();
#ifdef CMP_CC1101
	MSG_PRINTLN(F("CCInit "));
	uint8_t statRadio;
	for (radionr = 0; radionr < 4; radionr++) {		// init radio
		statRadio = tools::EEread(addr_statRadio + radionr);
		if (statRadio == 0xFF) {
			radio_bank[radionr] = 0xFF;
			continue;
		}
		statRadio = radioDetekt(false, statRadio);
		if (statRadio < 10) {				// ist dem Radio eine Bank zugeordnet?
			bankOffset = getBankOffset(statRadio);
			cc1101::CCinit_reg();
		}
		radio_bank[radionr] = statRadio;
		if (statRadio != tools::EEread(addr_statRadio + radionr)) {
			tools::EEwrite(addr_statRadio+radionr,statRadio);
			tools::EEstore();
		}
	}
	
//	if (radio_bank[remRadionr] < 10)
//	{
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
//	} 
//	else
//		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
#endif 

	if (musterDec.MdebEnabled) {
		MSG_PRINTLN(F("Starting timerjob"));
	}
	delay(50);

#ifdef MAPLE_Mini
	TIM_TypeDef *Instance = TIM1;
	HardwareTimer *MyTim = new HardwareTimer(Instance);
	MyTim->setMode(2, TIMER_OUTPUT_COMPARE);
	MyTim->setOverflow(31*1000, MICROSEC_FORMAT);
	MyTim->attachInterrupt(cronjob);
	MyTim->resume();
#else
	Timer1.initialize(31*1000); //Interrupt wird jede 31 Millisekunden ausgeloest
	Timer1.attachInterrupt(cronjob);
#endif
	cmdstring.reserve(maxCmdString);

	hasCC1101 = true;

	for (radionr = 0; radionr < 4; radionr++) {	// enableReceive bei allen korrekt erkannten radios denen eine Bank zugeordnet ist
		statRadio = radio_bank[radionr];
		if (statRadio < 10) {
			bankOffset = getBankOffset(statRadio);
			ccmode = tools::EEbankRead(addr_ccmode);
			if (radionr != 1 || ccmode > 0 || cc1101::regCheck()) {
				en_dis_receiver(true);
				if (radionr == 1 && ccmode == 0) {
					pinAsOutput(PIN_SEND);
				}
			}
			else {
				MSG_PRINT(F("cc1101 "));
				MSG_WRITE('A' + radionr);
				MSG_PRINT(F(" is for OOK not correctly set. "));
			}
		}
		else if (statRadio == 0x1F) {	// Init
			RXenabled[radionr] = true;
		}
	}
	MSG_PRINTLN("");
	getSelRadioBank();
}

#ifdef MAPLE_Mini
void cronjob(HardwareTimer*) {
	noInterrupts();
#else
void cronjob() {
	cli();
#endif
	static uint16_t cnt0 = 0;
	static uint8_t cnt1 = 0;
	 const unsigned long  duration = micros() - lastTime;
	 
	 if (duration > maxPulse && RXenabled[radioOokAsk]) { //Auf Maximalwert pruefen.
		 int16_t sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			 sDuration = -sDuration;
		 }
		 FiFo.enqueue(sDuration);

		 lastTime = micros();
	 }
	 digitalWrite(PIN_LED, blinkLED);
	 blinkLED = false;

#ifdef MAPLE_Mini
	interrupts();
#else
	sei();
#endif
	
	if (cnt0++ == 0) {
		if (cnt1++ == 0) {
			getUptime();
		}
	}

}


void loop() {
	static int16_t aktVal=0;
	bool state;
	uint8_t fifoCount;
	
#ifdef __AVR_ATmega32U4__
	serialEvent();
#endif
#ifdef MAPLE_Mini
	serialEvent();
#endif
#ifdef LAN_WIZ
	ethernetLoop();
#endif
	if (command_available) {
		command_available=false;
		HandleCommand();
		if (!command_available) { cmdstring = ""; }
		if (LEDenabled) {
		  blinkLED=true;
		}
	}
#ifdef WATCHDOG
	wdt_reset();
#endif

 uint8_t remRadionr = radionr;
 uint8_t remccmode = ccmode;
 uint8_t tmpBank;
 uint16_t bankoff;
 for (radionr = 0; radionr < 4; radionr++) {
  if (radio_bank[radionr] > 9) {
    continue;
  }
  tmpBank = radio_bank[radionr];
  bankoff = getBankOffset(tmpBank);
  ccmode = tools::EEread(bankoff + addr_ccmode);
  if (ccmode == 0) {
	musterDec.printMsgSuccess = false;
	while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder uebergeben

		aktVal=FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (musterDec.MdebEnabled && musterDec.printMsgSuccess) {
			fifoCount = FiFo.count();
			if (fifoCount > MdebFifoLimit) {
					MSG_PRINT(F("MF="));
					MSG_PRINTLN(fifoCount, DEC);
					//MSG_PRINTLN("");
			}
		}
		if (musterDec.printMsgSuccess && LEDenabled) {
			blinkLED=true; //LED blinken, wenn Meldung dekodiert
		}
		musterDec.printMsgSuccess = false;
	}
  }
  else if (ccmode < 15) {
	getRxFifo(bankoff);
  }
 }
 radionr = remRadionr;
 ccmode = remccmode;
}

void getRxFifo(uint16_t Boffs) {
	uint8_t fifoBytes;
	bool dup;		// true bei identischen Wiederholungen bei readRXFIFO

	if (isHigh(pinReceive[radionr])) {  // wait for CC1100_FIFOTHR given bytes to arrive in FIFO
		if (LEDenabled) {
			blinkLED=true;
		}
		if (ccmode == 4) {
			cc1101::ccStrobe_SIDLE();	// start over syncing
		}
			
		fifoBytes = cc1101::getRXBYTES(); // & 0x7f; // read len, transfer RX fifo
		if (fifoBytes > 0) {
			uint8_t marcstate;
			uint8_t RSSI = cc1101::getRSSI();
			
			if (ccmode == 9) {
				MSG_PRINT(F("RX("));
				MSG_PRINT(fifoBytes);
				MSG_PRINT(F(") "));
			}
			if (fifoBytes < 0x80) {	// RXoverflow?
				if (fifoBytes > ccMaxBuf) {
					fifoBytes = ccMaxBuf;
				}
				dup = cc1101::readRXFIFO(fifoBytes);
				if (ccmode != 2 || dup == false) {
					if (ccmode != 9) {
						MSG_PRINT(MSG_START);
						MSG_PRINT(F("MN;D="));
					}
					for (uint8_t i = 0; i < fifoBytes; i++) {
						printHex2(ccBuf[radionr][i]);
						//MSG_PRINT(" ");
					}
					if (ccmode == 9) {
						MSG_PRINT(F(" ("));
						MSG_PRINT(cc1101::getRXBYTES());
						MSG_PRINT(F(")"));
					}
					else {
						uint8_t n = tools::EEread(Boffs + addr_ccN);
						if (n > 0) {
							MSG_PRINT(F(";N="));
							MSG_PRINT(n);
						}
						MSG_PRINT(F(";R="));
						MSG_PRINT(RSSI);
						MSG_PRINT(F(";"));
						MSG_PRINT(MSG_END);
						MSG_PRINT("\n");
					}
				}
			}
			if (ccmode == 4) {
				switch (cc1101::getMARCSTATE()) {
					// RX_OVERFLOW
				case 17:
					// IDLE
				case 1:
					cc1101::ccStrobe_SFRX();	// Flush the RX FIFO buffer
					cc1101::ccStrobe_SIDLE();	// Idle mode
					cc1101::ccStrobe_SNOP();	// No operation
					cc1101::ccStrobe_SRX();	// Enable RX
					break;
				}
			}
			else {
				marcstate = cc1101::getMARCSTATE();
				if (ccmode == 9) {
					MSG_PRINT(F(" M"));
					MSG_PRINTLN(marcstate);
				}
				if (marcstate == 17 || ccmode == 3) {	// RXoverflow oder LaCrosse?
					if (cc1101::flushrx()) {		// Flush the RX FIFO buffer
						cc1101::setReceiveMode();
					}
				}
			}
		}
	}
}

//========================= Pulseauswertung ================================================
void handleInterrupt() {
#ifdef MAPLE_Mini
  noInterrupts();
#else
  cli();
#endif
  const unsigned long Time=micros();
  const unsigned long  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
	int16_t sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int16_t(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (isHigh(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration=-sDuration;
    }
	//MSG_PRINTLN(sDuration);
    FiFo.enqueue(sDuration);

    //++fifocnt;
  } // else => trash
#ifdef MAPLE_Mini
  interrupts();
#else
  sei();
#endif
}

void enableReceive() {
  if (RXenabled[radionr] == true) {
   if (ccmode == 0 && radionr == 1) {
     attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);
   }
   #ifdef CMP_CC1101
   if (hasCC1101 && ccmode < 15) {
     cc1101::ccStrobe_SIDLE();	// Idle mode
     delay(1);
     cc1101::setReceiveMode();
   }
   #endif
  }
}

void disableReceive() {
  if (ccmode == 0 && radionr == 1) {
    detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));
  }
  #ifdef CMP_CC1101
  if (hasCC1101) cc1101::ccStrobe_SIDLE();	// Idle mode
  #endif
  FiFo.flush();
}

/*void send_rawx(const uint8_t startpos,const uint16_t endpos,const int16_t *buckets, String *source=&cmdstring)
{
//  int16_t sendarr[] ={200,-200,300,-300,500,-400,600,-600,800,-800,500,-500,200,-200,300,-300,500,-400,600,-600,800,-800,500,-500,200,-200,500,-300,400,-400,500,-600,800,-800,500,-500,200,-200,300,-300,400,-400,600,-600,1100,-1100,800,-800,500,-500};
  int16_t sendarr[] = {-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-154,260,-1400,1724,-154,260,-367,260,-154,940,-572,260,-154,260,-154,1312,-154,940,-367,468,-1200,940,-1200,260,-784,260,-154,260};
  int16_t p;
  for (uint8_t i=0;i<62;i++ ) {
    p = sendarr[i];
    //MSG_PRINTLN(p);
    musterDec.decode(&p);
  }
}*/

//================================= RAW Send ======================================
void send_raw(const uint8_t startpos,const uint16_t endpos,const int16_t *buckets, String *source=&cmdstring)
{
	uint8_t index=0;
	unsigned long stoptime=micros();
	bool isLow;
	int16_t dur;

  if (ccmode == 0) {
	for (uint16_t i=startpos;i<=endpos;i++ )
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
		isLow=buckets[index] >> 15;
		dur = abs(buckets[index]); 		//isLow ? dur = abs(buckets[index]) : dur = abs(buckets[index]);
		while (stoptime > micros()){
			;
		}
		isLow ? digitalLow(PIN_SEND): digitalHigh(PIN_SEND);
		stoptime+=dur;
	}
	while (stoptime > micros()){
		;
	}
  } else {	// Send to Decoder
	for (uint16_t i=startpos;i<=endpos;i++ )
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
		isLow=buckets[index] >> 15;
		dur = abs(buckets[index]); 
		if (isLow) dur = -dur;
		musterDec.decode(&dur);
	}
  }
	//MSG_PRINTLN("");
}
//SM;R=2;C=400;D=AFAFAF;


void send_mc(const uint8_t startpos,const uint8_t endpos, const int16_t clock)
{
	int8_t b;
	char c;
	//digitalHigh(PIN_SEND);
	//delay(1);
	uint8_t bit;

	unsigned long stoptime =micros();
	for (uint8_t i = startpos; i <= endpos; i++) {
		c = cmdstring.charAt(i);
		b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

		for (bit = 0x8; bit>0; bit >>= 1) {
			for (byte i = 0; i <= 1; i++) {
				if ((i == 0 ? (b & bit) : !(b & bit)))
					digitalLow(PIN_SEND);
				else
					digitalHigh(PIN_SEND);
				
					stoptime += clock;
					while (stoptime > micros())
						;
			}
			
		}
		
	}
	// MSG_PRINTLN("");
}


bool split_cmdpart(int16_t *startpos, int16_t *startdata)
{
	int16_t endpos=0;
	*startdata = -1;
	endpos=cmdstring.indexOf(";",*startpos);     			 // search next   ";"

	if (endpos ==-1 || *startpos== -1) {
		*startdata = 0;
		return false;
	}
	msg_cmd0 = cmdstring.charAt(*startpos);
	msg_cmd1 = cmdstring.charAt(*startpos+1);
	/*MSG_PRINT("split_spos=");
	MSG_PRINT(*startpos);
	MSG_PRINT(" cmd=");
	MSG_PRINT(msg_cmd0);
	MSG_PRINTLN(msg_cmd1);*/

	if (msg_cmd0 == 'S') {
		*startdata = 0;
	} else if (msg_cmd0 == 'P') {
		if (cmdstring.charAt(*startpos+2) != '=') return false;
		*startdata = *startpos+3;
	} else {
		if (msg_cmd1 != '=') return false;
		*startdata = *startpos+2;
	}
	*startpos=endpos+1;    // Set startpos to endpos to extract next part
	return true;
}

// SC;R=4;SM;C=400;D=AFFFFFFFFE;SR;P0=-2500;P1=400;D=010;SM;D=AB6180;SR;D=101;
// SC;R=4;SM;C=400;D=FFFFFFFF;SR;P0=-400;P1=400;D=101;SM;D=AB6180;SR;D=101;
// SR;R=3;P0=1230;P1=-3120;P2=-400;P3=-900;D=030301010101010202020202020101010102020202010101010202010120202;
// SM;C=400;D=AAAAFFFF;F=123456;
// SR;R=10;P0=-2000;P1=-1000;P2=500;P3=-6000;D=2020202021212020202121212021202021202121212023;

struct s_sendcmd {
	int16_t sendclock;
	uint8_t datastart;
	uint16_t dataend;
	int16_t buckets[maxSendPattern];
	uint8_t repeats;
} ;

void send_cmd()
{
	#define combined 0
	#define manchester 1
	#define raw 2
	#define maxSendCmd 5
	
	uint16_t tmpBankoff;
	uint8_t remccmode = ccmode;
	uint8_t tmpBank;
	
	tmpBank = radio_bank[1];	// Modul B
	if (tmpBank > 9) {
		MSG_PRINTLN(F("Radio B is not active!"));
		return;
	}
	tmpBankoff = getBankOffset(tmpBank);
	ccmode = tools::EEread(tmpBankoff + addr_ccmode);
	if (ccmode != 0 && ccmode != 15) {
		MSG_PRINTLN(F("ASK/OOK send is only with ccmode 0 or 15 possible!"));
		ccmode = remccmode;
		return;
	}
	uint8_t remRadionr = radionr;
	radionr = 1;

	uint8_t repeats=1;  // Default is always one iteration so repeat is 1 if not set
	int16_t start_pos=0;
	int16_t startdata=0;
	uint8_t counter=0;
	bool isCombined = false;
	bool extraDelay = false;

	s_sendcmd command[maxSendCmd];
	command[0].datastart = 0;

	uint8_t ccParamAnz = 0;   // Anzahl der per F= uebergebenen cc1101 Register
	uint8_t val;

	disableReceive();

	uint8_t cmdNo=255;


	while (split_cmdpart(&start_pos,&startdata))
	{
		if (msg_cmd0 == 'S')
		{
			if (cmdNo == maxSendCmd) {
				startdata = -1;
				break;
			}
			if (msg_cmd1 == 'C')  // send combined informatio flag
			{
#ifdef DEBUGSENDCMD
				MSG_PRINTLN("SC");
#endif
				isCombined = true;
			}
			else if (msg_cmd1 == 'M' || msg_cmd1 == 'R') // send manchester or raw
			{
				cmdNo++;
				isCombined = false;
				command[cmdNo].sendclock = 0;
				command[cmdNo].repeats = 1;
#ifdef DEBUGSENDCMD
				MSG_PRINT("S");
				MSG_PRINTLN(msg_cmd1);
#endif
			}
		} else if (msg_cmd0 == 'R') {
			if (isCombined) {
				repeats = cmdstring.substring(startdata, start_pos-1).toInt();
#ifdef DEBUGSENDCMD
				MSG_PRINT("R=");
				MSG_PRINTLN(repeats);
#endif
			} else {
				if (cmdNo == 255) continue;
				command[cmdNo].repeats = cmdstring.substring(startdata, start_pos-1).toInt();
#ifdef DEBUGSENDCMD
				MSG_PRINT("R=");
				MSG_PRINTLN(command[cmdNo].repeats);
#endif
			}
		} else if (cmdNo == 255) {	// es wurde noch kein SR oder SM erkannt
			continue;
		} else if (msg_cmd0 == 'P') { // Do some basic detection if data matches what we expect
			counter = cmdstring.substring(startdata-2, startdata-1).toInt(); // extract the pattern number
			if (counter > maxSendPattern) {
				startdata = -1;
				break;
			}
			command[cmdNo].buckets[counter]=cmdstring.substring(startdata, start_pos-1).toInt();
#ifdef DEBUGSENDCMD
			MSG_PRINT("P");
			MSG_PRINT(counter);
			MSG_PRINT("=");
			MSG_PRINTLN(command[cmdNo].buckets[counter]);
#endif
		} else if (msg_cmd0 == 'D') {
			command[cmdNo].datastart = startdata;
			command[cmdNo].dataend = start_pos-2;
#ifdef DEBUGSENDCMD
			MSG_PRINT("D=");
			MSG_PRINTLN(cmdstring.substring(startdata, start_pos-1));
#endif
		} else if (msg_cmd0 == 'C') {
			command[cmdNo].sendclock = cmdstring.substring(startdata, start_pos-1).toInt();
			extraDelay = true;
#ifdef DEBUGSENDCMD
			MSG_PRINT("C=");
			MSG_PRINTLN(command[cmdNo].sendclock);
#endif
		} else if (msg_cmd0 == 'F') {
			ccParamAnz = (start_pos - startdata) / 2;
#ifdef DEBUGSENDCMD
			MSG_PRINT("F=");
#endif
			if (ccParamAnz > 0 && ccParamAnz <= 5 && hasCC1101 && ccmode == 0) {
				uint8_t hex;
				//MSG_PRINTLN("write new ccreg  ");
				for (uint8_t i=0;i<ccParamAnz;i++)
				{
					hex = (uint8_t)cmdstring.charAt(startdata + i*2);
					val = tools::hex2int(hex) * 16;
					hex = (uint8_t)cmdstring.charAt(startdata+1 + i*2);
					val = tools::hex2int(hex) + val;
					cc1101::writeReg(0x0d + i, val);            // neue Registerwerte schreiben
  #ifdef DEBUGSENDCMD
					printHex2(val);
  #endif
				}
			}
#ifdef DEBUGSENDCMD
			MSG_PRINTLN("");
#endif
		}
	}

	if (startdata == -1 || command[0].datastart == 0)
	{
		MSG_PRINT(F("send failed!"));
	} else
	{
		#ifdef CMP_CC1101
		if (hasCC1101 && ccmode == 0) cc1101::setTransmitMode();	
		#endif
		for (uint8_t i=0;i<repeats;i++)
		{
			for (uint8_t c=0;c<=cmdNo;c++)
			{
				if (command[c].sendclock == 0) {   // raw
					for (uint8_t rep = 0; rep < command[c].repeats; rep++) {
						send_raw(command[c].datastart, command[c].dataend, command[c].buckets);
					}
				} else {
					for (uint8_t rep = 0; rep < command[c].repeats; rep++) {
						if (rep > 0) delay(1);
						send_mc(command[c].datastart, command[c].dataend, command[c].sendclock);
					}
				}
				digitalLow(PIN_SEND);
			}
			//if (extraDelay) delay(1);
		}

	  if (ccmode == 0) {	//normal send
		if (cmdstring.length() <= maxSendEcho) {
			MSG_PRINT(cmdstring); // echo
		}
		else {
			MSG_PRINT(cmdstring.substring(0, maxSendEcho)); // echo
			MSG_PRINT(F(".."));
		}
		if (ccParamAnz > 0) {
			MSG_PRINT(F("ccreg write back "));
			for (uint8_t i=0;i<ccParamAnz;i++)
			{
				val = tools::EEbankRead(0x0f + i);
				printHex2(val);
				cc1101::writeReg(0x0d + i, val);    // gemerkte Registerwerte zurueckschreiben
			}
			//MSG_PRINTLN("");
		}
	  }
	  else {
		int16_t dur = -32001;
		musterDec.decode(&dur);
	  }
	}
	
//#ifndef SENDTODECODER
	enableReceive();	// enable the receiver
	MSG_PRINTLN("");
//#endif
	ccmode = remccmode;
	radionr = remRadionr;
}


void send_ccFIFO()
{
	uint8_t repeats=1;  // Default is always one iteration so repeat is 1 if not set
	int8_t startpos=0;
	int8_t endpos=0;
	int8_t startn=0;
	int8_t sendN=0;
	int8_t startdata=0;
	uint8_t enddata=0;
	
	startpos = cmdstring.indexOf(";R=",2);
	
	startn = cmdstring.indexOf(";N=",2);
	
	startdata = cmdstring.indexOf(";D=",2);
	if (startdata > 0 && startn > 0) {		// D= und N= gefunden
		endpos = startn;
		sendN = cmdstring.substring(startn+3, startdata).toInt();
		uint8_t remRadionr = radionr;
		uint8_t remccmode = ccmode;
		if (sendN > 0) {
			uint16_t bankoff;
			for (radionr = 0; radionr < 4; radionr++) {	// die den radio zugeordneten Baenke nach sendN durchsuchen
				if (radio_bank[radionr] > 9) {
					continue;
				}	
				bankoff = getBankOffset(radio_bank[radionr]);
				if (sendN == tools::EEread(bankoff + addr_ccN)) {
					ccmode = tools::EEread(bankoff + addr_ccmode);
					break;
				}
			}
		}
		else {
			radionr = 4;
		}
		
	  if (radionr < 4) {
		if (startpos > 0) {
			repeats = cmdstring.substring(startpos+3, endpos).toInt();
			if (repeats > 50) {
				repeats = 50;
			}
		}
		startdata += 3;
		
		/*MSG_PRINT(F("repeats="));
		MSG_PRINT(repeats);
		MSG_PRINT(F(" N="));
		MSG_PRINT(sendN);
		MSG_PRINT(F(" senddat="));
		MSG_PRINTLN(cmdstring.substring(startdata));*/
		enddata = cmdstring.indexOf(";",startdata);		// search next   ";"
		//MSG_PRINT(F(" end="));
		//MSG_PRINTLN(enddata);
		if (enddata > startdata) {
			disableReceive();
			for (uint8_t i = 0; i < repeats; i++) {
				if (cc1101::setTransmitMode() == false) {
					startdata = -1;
					break;
				}
				cc1101::sendFIFO(startdata, enddata);
			}
			MSG_PRINT(cmdstring); // echo
			MSG_PRINT(F("Marcs="));
			MSG_PRINTLN(cc1101::getMARCSTATE());
			enableReceive();
		}
		else {
			startdata = -1;
		}
	  }
	  else {
		MSG_PRINT(F("N not found, "));
		startdata = -1;
	  }
	  radionr = remRadionr;
	  ccmode = remccmode;
	}
	if (startdata == -1 || startn == -1)
	{
		MSG_PRINTLN(F("send failed!"));
	}
}

//================================= Kommandos ======================================
void IT_CMDs();

void HandleCommand()
{
//	uint8_t reg;
//	uint8_t val;
	uint8_t i;
	
	for (i=0; i < cmdAnz; i++) {
		if (cmdstring.charAt(0) == cmd0[i]) {
			if (cmd1[i] == ' ' || (cmdstring.charAt(1) == cmd1[i])) {
				break;
			}
		}
	}
	//MSG_PRINT(i);
	unsuppCmd = false;
	CmdOk = false;
	if (i < cmdAnz) {
		//MSG_PRINT(F(" "));
		//MSG_PRINT(cmd0[i]);
		//MSG_PRINT(cmd1[i]);
		cmdFP[i]();
	}
	else {
		unsuppCmd = true;
	}
	//MSG_PRINTLN("");
	
	if (unsuppCmd) {
		MSG_PRINTLN(F("Unsupported command"));
	}
	else if (CmdOk) {
		MSG_PRINTLN(F("ok"));
	}
}



void cmd_help_S()	// get help configvariables
{
	char buffer[12];
	for (uint8_t i = 0; i < CSetAnz; i++) {
	    strcpy_P(buffer, (char*)pgm_read_ptr(&(CSetCmd[i])));
	    MSG_PRINT(F("CS"));
	    MSG_PRINT(buffer);
	    MSG_PRINT(F("= "));
	}
	MSG_PRINTLN("");
}

void cmd_help()
{
	MSG_PRINT(F("? Use one of "));
	for (uint8_t i = 0; i < cmdAnz; i++) {
		if (hasCC1101 || cmdCC[i] == 0) {
			MSG_PRINT(cmd0[i]);
			if (cmd1[i] != ' ') {
				MSG_PRINT(cmd1[i]);
			}
			MSG_PRINT(F(" "));
		}
	}
	MSG_PRINTLN("");
}

void cmd_bank()
{
	uint8_t posDigit = 1;
	uint8_t remRadionr = 255;
	if (cmdstring.charAt(1) >= 'A' && cmdstring.charAt(1) <= 'D') {		// Radio A-D
		remRadionr = radionr;	// Radionr merken
		radionr = (uint8_t)cmdstring.charAt(1) - 65;
		//uint8_t statRadio = tools::EEread(addr_statRadio + radionr);
		if (radio_bank[radionr] > 0x1F) {
			MSG_PRINTLN(F("radio is not aktive!"));
			radionr = remRadionr;
			return;
		}
		else {
			posDigit = 2;
		}
	}
	
	if (isDigit(cmdstring.charAt(posDigit))) {	// Bank 0-9
		uint8_t digit;
		digit = (uint8_t)cmdstring.charAt(posDigit);
		uint8_t remBank = bank;		// bank merken
		bank = tools::hex2int(digit);
		if (posDigit == 2) {		// es wurde ein Radio angegeben
			for (uint8_t i = 0; i < 4; i++) {
				if (i == radionr) continue;
				if ((radio_bank[i] & 0x0F) == bank) {	// testen ob die ausgewaehlte Bank noch frei ist
					MSG_PRINT(F("Bank "));
					MSG_PRINT(bank);
					MSG_PRINT(F(" is already used by radio "));
					MSG_WRITE(i + 'A');
					MSG_PRINTLN("");
					bank = remBank;
					radionr = remRadionr;
					remRadionr = 255;
					break;
				}
			}
			if (remRadionr == 255) {	// Abbruch
				return;
			}
		}
		radio_bank[radionr] = bank;
		bankOffset = getBankOffset(bank);
		if (bank == 0 || cmdstring.charAt(0) == 'e' || (tools::EEbankRead(0) == bank && tools::EEbankRead(1) == (255 - bank))) {
			if (cmdstring.charAt(posDigit+1) == 'W') {
				tools::EEwrite(addr_statRadio+radionr, bank);
				tools::EEstore();
				MSG_PRINT(F("write "));
			}
			MSG_PRINT(F("set "));
			//ccN = tools::EEbankRead(addr_ccN);
			ccmode = tools::EEbankRead(addr_ccmode);
			if (ccmode == 255) {
				ccmode = 0;
				tools::EEbankWrite(addr_ccmode, ccmode);
				tools::EEstore();
			}
			if (cmdstring.charAt(0) != 'e') {
				print_Bank();
				cc1101::CCinit();
				setCCmode();
			}
		}
		else {
			MSG_PRINT(F("The bank "));
			MSG_PRINT(bank);
			MSG_PRINT(F(" was not complete initialized, therefore the bank and radio is reseted to sduino defaults (raw e). "));
			cmd_ccFactoryReset();
		}
	}
	else if (posDigit == 2) {		// es wurde ein Radio angegeben und keine bank angegeben -> das angegebene radio wird das aktuelle
		if (radio_bank[radionr] < 10) {
			bank = radio_bank[radionr];
			bankOffset = getBankOffset(bank);
			//ccN = tools::EEbankRead(addr_ccN);
			ccmode = tools::EEbankRead(addr_ccmode);
			MSG_PRINT(F("switch to radio "));
			MSG_WRITE(radionr + 'A');
			if (cmdstring.charAt(posDigit) == 'W') {
				tools::EEwrite(addr_selRadio, radionr);
				tools::EEstore();
				MSG_PRINT(F(" and write"));
			}
			MSG_PRINTLN("");
		}
		else {
			MSG_PRINTLN(F("Error! radio has no bank"));
			radionr = remRadionr;
		}
	}
	else if (posDigit == 1 && cmdstring.charAt(0) == 'b' && (cmdstring.length() == 1 || cmdstring.charAt(1) == '?')) {
		print_Bank();
	}
	else if (posDigit == 1 && cmdstring.charAt(1) == 's') {
		print_bank_sum();
	}
	else if (posDigit == 1 && cmdstring.charAt(1) == 'r') {
		print_radio_sum();
	}
	else {
		unsuppCmd = true;
	}
}

uint16_t getBankOffset(uint8_t tmpBank)
{
	uint16_t bankOffs;
	if (tmpBank == 0) {
		bankOffs = 0;
	}
	else {
		bankOffs = 0x100 + ((tmpBank - 1) * 0x40);
	}
	return bankOffs;
}

void print_ccconf(uint16_t bankOffs)
{
	char hexString[6];
	
	MSG_PRINT(F(" sync="));
	printHex2(tools::EEread(bankOffs + 2 + CC1101_SYNC1));
	printHex2(tools::EEread(bankOffs + 2 + CC1101_SYNC0));
	MSG_PRINT(F(" ccconf="));
	for (uint8_t i = 0x0F; i <= 0x1F; i++) {
		printHex2(tools::EEread(bankOffs + i));
	}
	MSG_PRINT(F(" boffs="));
	sprintf(hexString, "%04X", bankOffs);
	MSG_PRINT(hexString);
}

void print_Bank()
{
	uint8_t tmp_ccN = tools::EEbankRead(addr_ccN);
	MSG_PRINT(F("r="));
	MSG_WRITE('A' + radionr);
	MSG_PRINT(F(" b="));
	MSG_PRINT(bank);
	if (RXenabled[radionr] == false) {
		MSG_PRINT(F(" rx=0"));
	}
	if (tmp_ccN > 0) {
		MSG_PRINT(F(" N="));
		MSG_PRINT(tmp_ccN);
	}
	MSG_PRINT(F(" ccmode="));
	MSG_PRINT(ccmode);
	
	print_ccconf(bankOffset);
	MSG_PRINTLN("");
}

void print_bank_sum()	// bs - Banksummary
{
	char bankStr[23];
	char radioStr[23];
	char Nstr[23];
	char ccmodeStr[23];
	uint16_t sBankoff;
	uint8_t sCcmode;
	uint8_t i;
	uint8_t i2;
	uint8_t j;
	
	for (i = 0; i <= 9; i++) {
		i2 = i * 2;
		bankStr[i2] = '0' + i;
		bankStr[i2+1] = ' ';
		
		for (j = 0; j < 4; j++) {
			radioStr[i2] = '-';
			radioStr[i2+1] = ' ';
			if (i == radio_bank[j]) {
				radioStr[i2] = 'A' + j;
				if (j == radionr) {
					radioStr[i2+1] = '*';
				}
				break;
			}
		}
		
		sBankoff = getBankOffset(i);
		if ((tools::EEread(sBankoff) == i && tools::EEread(sBankoff+1) == (255 - i)) || i == 0) {
			Nstr[i2] = '0' + tools::EEread(sBankoff + addr_ccN);
			sCcmode = tools::EEread(sBankoff + addr_ccmode);
			if (sCcmode < 10) {
				ccmodeStr[i2] = '0' + sCcmode;
			}
			else {
				ccmodeStr[i2] = 'A' + sCcmode - 10;
			}
		}
		else {
			Nstr[i2] = '-';
			ccmodeStr[i2] = '-';
		}
		Nstr[i2+1] = ' ';
		ccmodeStr[i2+1] = ' ';
	}
	bankStr[20] = 0;
	radioStr[20] = 0;
	Nstr[20] = 0;
	ccmodeStr[20] = 0;
	
	MSG_PRINT(F("Bank__ "));
	MSG_PRINT(bankStr);
	MSG_PRINT(F(" Radio_ "));
	MSG_PRINT(radioStr);
	if (radioStr[19] == '*') {	// wenn * am Ende, dann zusaetzliches Leerzeichen einfuegen damit es wieder 2 Leerzeichen sind.
		MSG_PRINT(F(" "));
	}
	MSG_PRINT(F(" N_____ "));
	MSG_PRINT(Nstr);
	MSG_PRINT(F(" ccmode "));
	MSG_PRINT(ccmodeStr);
	MSG_PRINT(F("  "));
	
	uint8_t ch;
	for (i = 0; i <= 9; i++) {
		i2 = i * 2;	
		if (Nstr[i2] != '-') {		// Bank Aktiv?
			if (ccmodeStr[i2] == '0') {
				strcpy(bankStr, "SlowRF");
				j = 6;
			}
			else {
				for (j = 0; j < 8; j++) {
					ch = tools::EEread(0x40 + (i * 8) + j);
					if ((ch >= 32 && ch <= 122) || ch == 0) {	// space to z
						bankStr[j] = ch;
						if (ch == 0) {	// end
							break;
						}
					}
					else {	// kein gueltiges Zeichen
						j = 0;
						break;
					}
				}
			}
			if (j > 3) {
				bankStr[8] = 0;
				MSG_PRINT(F(" "));
				MSG_PRINT(i);		// BankNr
				MSG_PRINT(F(" - "));
				MSG_PRINT(bankStr);
				MSG_PRINT(F(" "));
			}
		}
	}
	MSG_PRINTLN("");
		
	//	Bank__ 0 1 2 3 4 5 6 7 8 9  Radio_ B A - C - - - - - -  N_____ 0 0 2 3 4 - - - - -  ccmode 0 3 3 3 2 - - - - -
}

void print_radio_sum()	// br - Bankinfo fuer alle cc1101 denen eine Bank zugeordnet ist, ausgeben
{
	uint16_t rbankoff;
	uint8_t rbank;
	uint8_t rccmode;
	uint8_t rN;
	bool twospaceFlag = false;
	
	for (uint8_t i = 0; i < 4; i++) {
		if (radio_bank[i] > 9) {
			continue;
		}	
		rbank = radio_bank[i];
		rbankoff = getBankOffset(rbank);
		rccmode = tools::EEread(rbankoff + addr_ccmode);
		rN      = tools::EEread(rbankoff + addr_ccN);
		if (twospaceFlag) {
			MSG_PRINT(F("  "));
		}
		else {
			twospaceFlag = true;
		}
		MSG_PRINT(F("r="));
		MSG_WRITE('A' + i);
		MSG_PRINT(F(" b="));
		MSG_PRINT(rbank);
		if (RXenabled[i] == false) {
			MSG_PRINT(F(" rx=0"));
		}
		if (rN > 0) {
			MSG_PRINT(F(" N="));
			MSG_PRINT(rN);
		}
		MSG_PRINT(F(" ccmode="));
		MSG_PRINT(rccmode);
	
		print_ccconf(rbankoff);
	}
	MSG_PRINTLN("");
}

void print_mac(uint8_t mac[])
{
	for (uint8_t i = 0; i < 6; i++) {
		printHex2(mac[i]);
		if (i < 5) {
			MSG_PRINT(F(":"));
		}
	}
}

void print_ip(uint8_t ip[])
{
	for (uint8_t i = 0; i < 4; i++) {
		MSG_PRINT(ip[i]);
		if (i < 3) {
			MSG_PRINT(F("."));
		}
	}
}

void cmd_Version()	// V: Version
{
	MSG_PRINT(F("V " PROGVERS " SIGNALduino "));
#ifdef LAN_WIZ
	MSG_PRINT(F("LAN "));
#endif
	if (hasCC1101) {
	    MSG_PRINT(F("cc1101 "));
#ifdef PIN_MARK433
	    MSG_PRINT(F("(minicul "));
	    MSG_PRINT(isLow(PIN_MARK433) ? "433" : "868");
	    MSG_PRINT(F("MHz) "));
#endif
	}
	MSG_PRINT(F("(R:"));
	
	uint8_t statRadio;
	for (uint8_t i = 0; i < 4; i++) {
		statRadio = radio_bank[i];
		if (statRadio != 0xFF) {
			MSG_PRINT(F(" "));
			MSG_WRITE(i + 'A');
			if (statRadio & 0x40) {	// Bit6 = 1  Init failed
				MSG_PRINT(F("-"));
			}
			else if (statRadio == 0x1F) {	// Bit4 = 1  Init
				MSG_PRINT(F("i"));
			}
			else {
				MSG_PRINT(statRadio & 0x0F);
			}
			if (i == radionr) {
				MSG_PRINT(F("*"));
			}
		}
	}
	
	MSG_PRINT(F(") "));
	MSG_PRINTLN(F("- compiled at " __DATE__ " " __TIME__));
}

void cmd_freeRam()	// R: FreeMemory
{
#ifdef MAPLE_Mini
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();
	MSG_PRINTLN(((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks);
#else
	MSG_PRINTLN(freeRam());
#endif
}

void cmd_send()
{
	if (musterDec.getState() != searching )
	{
		command_available=true;
	} else {
		if (cmdstring.charAt(1) != 'N') {
			send_cmd(); // Part of Send
		}
		else {
			send_ccFIFO();
		}
	}
}

void cmd_uptime()	// t: Uptime
{
	MSG_PRINTLN(getUptime());
}

void cmd_toggleBank()	// T<bank><sec>
{
	uint8_t setBank;
	uint8_t sec;
	uint16_t bankOffs;
	
	if (cmdstring.charAt(1) == '?') {
		MSG_PRINT(F("Toggle="));
		for (uint8_t i = 0; i <= 9; i++) {
			bankOffs = getBankOffset(i);
			MSG_PRINT(F(" "));
			sec = tools::EEbankRead(addr_togglesec);
			if (sec > 99) {
				sec = 0;
				tools::EEbankWrite(addr_togglesec, sec);
				tools::EEstore();
			}
			MSG_PRINT(sec * 10);
		}
		MSG_PRINTLN("");
	}
	else if (isDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isDigit(cmdstring.charAt(3))) {
		uint8_t high;
		uint8_t digit;
		
		setBank = (uint8_t)cmdstring.charAt(1);
		setBank = tools::hex2int(setBank);
		bankOffs = getBankOffset(setBank);
		MSG_PRINT(F("setToggle b="));
		MSG_PRINTLN(setBank);
		digit = (uint8_t)cmdstring.charAt(2);
		high = tools::hex2int(digit);
		digit = (uint8_t)cmdstring.charAt(3);
		sec = high * 10 + tools::hex2int(digit);
		tools::EEbankWrite(addr_togglesec, sec);
		tools::EEstore();
		MSG_PRINT(F(" sec="));
		MSG_PRINTLN(sec * 10);
	}
	else {
		unsuppCmd = true;
	}
	
}

void ccRegWrite()	// CW cc register write
{
	int16_t pos;
	uint8_t val;
	uint8_t reg;
	uint8_t i;
	uint8_t tmp_ccN = 0xFF;
	uint8_t tmp_ccmode = 0xFF;
	bool flag = false;
	
	pos = 2;
	for (i=0; i<64; i++)
	{
		if (!isHexadecimalDigit(cmdstring.charAt(pos)) || !isHexadecimalDigit(cmdstring.charAt(pos+1)) || !isHexadecimalDigit(cmdstring.charAt(pos+2)) || !isHexadecimalDigit(cmdstring.charAt(pos+3))) {
			break;
		}
		reg = tools::cmdstringPos2int(pos);
		if (reg > 0x47) {
			break;
		}
		val = tools::cmdstringPos2int(pos+2);
		if (reg <= 0x2F) {
			cc1101::writeReg(reg,val);
			if (reg <= 0x28) {
				reg += 2;
			}
			else {	// 0x29 - 0x2F nur fuer Testzwecke
				reg = 0x2F;
			}
		}
		else if (reg == addr_ccmode) {
			tmp_ccmode = val;
		}
		else if (reg == addr_ccN) {
			tmp_ccN = val;
		}
		if (reg < 0x40) {
			if (reg != 0x2F) {	// bei Testregister kein write
				tools::EEbankWrite(reg, val);
				if (reg == 0x37) {		// Ende der patable
					cc1101::writePatable();
				}
			}
		}
		else {		// 0x40 - 0x47  Bank Kurzbeschreibung (max 8 Zeichen)
			reg = reg + (bank * 8);
			tools::EEwrite(reg, val);
		}
		/*MSG_PRINT(F("reg="));
		printHex2(reg);
		MSG_PRINT(F(" val="));
		printHex2(val);
		MSG_PRINTLN("");*/
		pos = cmdstring.indexOf(",",pos);
		if (pos == -1) {
			flag = true;
			break;
		}
		pos++;
	}
	tools::EEstore();
	MSG_PRINT(cmdstring); // echo
	if (flag) {
		MSG_PRINT(F(" ok"));
		if (tmp_ccN != 0xFF) {
			MSG_PRINT(F(",N="));
			MSG_PRINT(tmp_ccN);
			//ccN = tmp_ccN;
		}
		if (tmp_ccmode != 0xFF) {
			//cc1101::setIdleMode();
			//cc1101::setReceiveMode();
			MSG_PRINT(F(",ccmode="));
			MSG_PRINT(tmp_ccmode);
			ccmode = tmp_ccmode;
			setCCmode();
		}
		MSG_PRINTLN("");
	} else {
		MSG_PRINT(F(" Error at pos="));
		MSG_PRINTLN(pos);
	}
}

void cmd_config()	// C read ccRegister
{
	uint8_t reg;
	
	if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {
		reg = tools::cmdstringPos2int(1);
		cc1101::readCCreg(reg);
	}
	else {
		unsuppCmd = true; 
	}
}

void cmd_writeEEPROM()	// write EEPROM und CC11001 register
{
	uint8_t val;
	uint8_t reg;
	bool ret = false;
	
    if (cmdstring.charAt(1) == 'S' && cmdstring.charAt(2) == '3' && hasCC1101) {       // WS<reg>  Command Strobes
        cc1101::commandStrobes();
    } else if (cmdstring.charAt(1) == 'i') {	// write ip
        if (cmdstring.charAt(2) == 'a') {	// adress
            ret = tools::cmdstringPos2ip(ip, 3, EE_IP4_ADDR);
        }
        else if (cmdstring.charAt(2) == 'g') {	// gateway
            ret = tools::cmdstringPos2ip(gateway, 3, EE_IP4_GATEWAY);
        }
        else if (cmdstring.charAt(2) == 'n') {	// netmask
            ret = tools::cmdstringPos2ip(netmask, 3, EE_IP4_NETMASK);
        }
        else {
           unsuppCmd = true;
           return;
        }
        if (ret == false) {
           MSG_PRINTLN(F("incorrect"));
        }
        else {
          CmdOk = true;
        }
    } else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4))) {
         reg = tools::cmdstringPos2int(1);
         val = tools::cmdstringPos2int(3);
         if (reg < 0x40) {
           tools::EEbankWrite(reg, val);
           if (reg > 1 && reg < 0x30) {
             if (hasCC1101) {
               cc1101::writeReg(reg-2, val);
             }
           }
         }
         else {   // ab 0x40 immer in Bank 0
           tools::EEwrite(reg, val);
         }
         tools::EEstore();
         MSG_PRINT("W");
         printHex2(reg);
         printHex2(val);
         MSG_PRINTLN("");
    } else {
         unsuppCmd = true;
    }
}

void cmd_readEEPROM()	// R<adr>  read EEPROM
{
	// rN<adr16>  read 64 Byte from EEPROM 
	if (cmdstring.charAt(1) == 'N' && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4)) && isHexadecimalDigit(cmdstring.charAt(5))) {
		uint16_t addr;
		uint8_t high;
		uint8_t low;
		char hexString[6];
		
		high = tools::cmdstringPos2int(2);
		low = tools::cmdstringPos2int(4);
		addr = high * 256 + low;
		
		for (uint8_t j = 0; j < 4; j++) {
			sprintf(hexString, "%04X", addr);
			MSG_PRINT(F("EEPROM "));
			MSG_PRINT(hexString);
			MSG_PRINT(F(":"));
			for (uint8_t i = 0; i < 16; i++) {
				MSG_PRINT(F(" "));
				printHex2(tools::EEread(addr + i));
			}
			addr += 16;
			MSG_PRINT(F("  "));
		}
		MSG_PRINTLN("");
		return;
	}
	if (cmdstring.charAt(1) == 'i') {	// print ethernet config
		MSG_PRINT(F("mac = "));
		print_mac(mac);
		MSG_PRINT(F("    ip = "));
		print_ip(ip);
		MSG_PRINT(F("  gw = "));
		print_ip(gateway);
		MSG_PRINT(F("  nm = "));
		print_ip(netmask);
		MSG_PRINTLN("");
		return;
	}
   if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {             // r<adr>  read EEPROM
     uint8_t reg;
     uint16_t reg16;
     
     reg = tools::cmdstringPos2int(1);
     MSG_PRINT(F("EEPROM "));
     printHex2(reg);
     reg16 = (uint16_t)reg;
     if (reg < 0x40) {
        reg16 += bankOffset;
     }
     if (cmdstring.charAt(3) == 'n') {
         MSG_PRINT(F(" :"));
         for (uint8_t i = 0; i < 16; i++) {
             MSG_PRINT(F(" "));
             printHex2(tools::EEread(reg16 + i));
         }
     } else {
        MSG_PRINT(F(" = "));
        printHex2(tools::EEread(reg16));
     }
     MSG_PRINTLN("");
     return;
  }
     unsuppCmd = true;
}

void cmd_writePatable()
{
  uint8_t val;
  if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {
     val = tools::cmdstringPos2int(1);
     cc1101::writeCCpatable(val);
     MSG_PRINT(F("Write "));
     printHex2(val);
     MSG_PRINTLN(F(" to PATABLE done"));
  } else {
     unsuppCmd = true;
  }
}

void cmd_configFactoryReset()	// eC - initEEPROMconfig
{
	uint8_t statRadio;
	
	initEEPROMconfig();
	callGetFunctions();
	radionr = defSelRadio;
	statRadio = tools::EEread(addr_statRadio + defSelRadio);
	statRadio = radioDetekt(false, statRadio);
	if (statRadio == 0) {				// ist dem Radio Bank 0 zugeordnet?
		getSelRadioBank();
		cc1101::CCinit_reg();
		RXenabled[radionr] = true;
		setCCmode();
	}
	else {
		radio_bank[defSelRadio] = statRadio;
		if (statRadio != tools::EEread(addr_statRadio + defSelRadio)) {
			tools::EEwrite(addr_statRadio+radionr, defSelRadio);
			tools::EEstore();
		}
		getSelRadioBank();
	}
}

void cmd_ccFactoryReset()	// e<0-9>
{
         if (cmdstring.charAt(0) == 'e' && isDigit(cmdstring.charAt(1))) {
            cmd_bank();
         }
         cc1101::ccFactoryReset();
         cc1101::CCinit();
         tools::EEbankWrite(addr_ccN, 0);
         //ccN = 0;
         ccmode = 0;
         tools::EEbankWrite(addr_ccmode, ccmode);
         setCCmode();
         if (bank > 0) {
            tools::EEbankWrite(0, bank);
            tools::EEbankWrite(1, (255 - bank));
         }
         tools::EEstore();
         if (cmdstring.charAt(0) == 'e') {
            print_Bank();
         }
}

inline void getConfig()
{
  if (ccmode == 0 || ccmode == 15) {
   if (musterDec.MSeqEnabled == 0 || musterDec.MSenabled == 0) {
      MSG_PRINT(F("MS="));
      MSG_PRINT(musterDec.MSenabled,DEC);
   }
   else if (musterDec.MSenabled) {
      MSG_PRINT(F("MSEQ=1"));
   }
   MSG_PRINT(F(";MU="));
   MSG_PRINT(musterDec.MUenabled, DEC);
   MSG_PRINT(F(";MC="));
   MSG_PRINT(musterDec.MCenabled, DEC);
   MSG_PRINT(F(";Mred="));
   MSG_PRINT(musterDec.MredEnabled, DEC);
  }
  else {
    MSG_PRINT(F("ccmode="));
    MSG_PRINT(ccmode);
  }
   if (LEDenabled == false) {
      MSG_PRINT(F(";LED=0"));
   }
   if (RXenabled[radionr] == false) {
      MSG_PRINT(F(";RX=0"));
   }
   if (toggleBankEnabled == true) {
      MSG_PRINT(F(";toggleBank=1"));
   }
  if (ccmode == 0 || ccmode == 15) {
   MSG_PRINT(F("_MScnt="));
   MSG_PRINT(musterDec.MsMoveCountmax, DEC);
   if (musterDec.maxMuPrint < musterDec.maxMsgSize) {
      MSG_PRINT(F(";maxMuPrint="));
      MSG_PRINT(musterDec.maxMuPrint, DEC);
   }
   MSG_PRINT(F(";maxMsgSize="));
   MSG_PRINT(musterDec.maxMsgSize, DEC);
   if (musterDec.MuSplitThresh > 0) {
      MSG_PRINT(F(";MuSplitThresh="));
      MSG_PRINT(musterDec.MuSplitThresh, DEC);
   }
   if (musterDec.mcMinBitLen != mcMinBitLenDef) {
      MSG_PRINT(F(";mcMinBitLen="));
      MSG_PRINT(musterDec.mcMinBitLen, DEC);
   }
   if (musterDec.cMaxNumPattern != CSetDef[5]) {
      MSG_PRINT(F(";maxNumPat="));
      MSG_PRINT(musterDec.cMaxNumPattern, DEC);
   }
   if (musterDec.cMaxPulse != -maxPulse) {
      MSG_PRINT(F(";maxPulse="));
      MSG_PRINT(musterDec.cMaxPulse, DEC);
   }
   MSG_PRINT(F(";Mdebug="));
   MSG_PRINT(musterDec.MdebEnabled, DEC);
   if (musterDec.MdebEnabled) {
      MSG_PRINT(F(";MdebFifoLimit="));
      MSG_PRINT(MdebFifoLimit, DEC);
      MSG_PRINT(F("/"));
      MSG_PRINT(FIFO_LENGTH, DEC);
   }
  }
   MSG_PRINTLN("");
}


inline void configCMD()
{
  bool *bptr;

  if (cmdstring.charAt(2) == 'S') {  	  //MS
	bptr=&musterDec.MSenabled;
  }
  else if (cmdstring.charAt(2) == 'U') {  //MU
	bptr=&musterDec.MUenabled;
  }
  else if (cmdstring.charAt(2) == 'C') {  //MC
	bptr=&musterDec.MCenabled;
  }
  else if (cmdstring.charAt(2) == 'R') {  //Mreduce
	bptr=&musterDec.MredEnabled;
  }
  else if (cmdstring.charAt(2) == 'D') {  //Mdebug
	bptr=&musterDec.MdebEnabled;
  }
  else if (cmdstring.charAt(2) == 'L') {  //LED
	bptr=&LEDenabled;
  }
  else if (cmdstring.charAt(2) == 'Q') {  //MSeq
    bptr=&musterDec.MSeqEnabled;
  }
//  else if (cmdstring.charAt(2) == 'T') {  // toggleBankEnabled
//	bptr=&toggleBankEnabled;
//  }
  else {
	unsuppCmd = true;
	return;
  }

  if (cmdstring.charAt(1) == 'E') {   // Enable
	*bptr=true;
  }
  else if (cmdstring.charAt(1) == 'D') {  // Disable
	*bptr=false;
  } else {
	unsuppCmd = true;
	return;
  }
  storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled, musterDec.MredEnabled, musterDec.MdebEnabled, LEDenabled, musterDec.MSeqEnabled, toggleBankEnabled);
}


inline void configSET()
{ 
	char buffer[15];
	int16_t i = cmdstring.indexOf("=",4);
	uint8_t n = 0;
	uint8_t val;
	uint16_t val16;
	if (i < 0) {
		unsuppCmd = true;
	}
	while (n < CSetAnz) {
		strcpy_P(buffer, (char*)pgm_read_ptr(&(CSetCmd[n])));
		if (cmdstring.substring(2, i) == buffer) {
			MSG_PRINT(buffer);
			MSG_PRINT(F("="));
			if (n < CSet16) {
				val = cmdstring.substring(i+1).toInt();
				MSG_PRINTLN(val);
				if (n == CSccmode || n == CSccN) {		// ccmode und ccN wird in die EEPROM Speicherbank geschrieben
					tools::EEbankWrite(CSetAddr[n], val);
				} else {
					tools::EEwrite(CSetAddr[n], val);
				}
			}
			else {
				val16 = cmdstring.substring(i+1).toInt();
				MSG_PRINTLN(val16);
				val = (val16>>8) & 0xFF;
				tools::EEwrite(CSetAddr[n+(n-CSet16)], val);		// high
				val = val16 & 0xFF;
				tools::EEwrite(CSetAddr[n+(n-CSet16)+1], val);	// low
			}
			tools::EEstore();
			break;
		}
		n++;
	}
	
	if (n == 0) {  				// fifolimit
		MdebFifoLimit = val;
	}
	else if (n == 1) {			// mcmbl
		musterDec.mcMinBitLen = val;
	}
	else if (n == 2) {			// mscnt
		musterDec.MsMoveCountmax = val;
	}
	else if (n == 3) {			//maxMuPrint
		if (val == 0) {
			val = 1;
		}
		musterDec.maxMuPrint = val * 256;
	}
	else if (n == 4) {			// 
		if (val <=1 ) {
			musterDec.maxMsgSize = 254;
		}
		else if (val * 256 > defMaxMsgSize) {
			musterDec.maxMsgSize = defMaxMsgSize;
		}
		else {
			musterDec.maxMsgSize = val * 256;
		}
	}
	else if (n == 5) {			// maxnumpat
		musterDec.cMaxNumPattern = val;
	}
//	else if (n == CSccN) {			// ccN
//		ccN = val;
//	}
	else if (n == CSccmode) {			// ccmode
		ccmode = val;
		setCCmode();
	}
	else if (n == CSet16) {			// muthresh
		musterDec.MuSplitThresh = val16;
	}
	else if (n == CSet16+1) {			// maxpulse
		if (val16 != 0) {
			musterDec.cMaxPulse = -val16;
		}
		else {
			musterDec.cMaxPulse = -maxPulse;
		}
	}
	else if (n != CSccN) {
		unsuppCmd = true;
	}
}

inline uint8_t radioDetekt(bool confmode, uint8_t Dstat)
{
	uint8_t pn;
	uint8_t ver;

	MSG_PRINT(F("detect "));
	MSG_WRITE('A' + radionr);
	if (cc1101::CCreset()) {
		pn = cc1101::getCCPartnum();
		ver = cc1101::getCCVersion();
		MSG_PRINT(F(": Partn="));
		MSG_PRINT(pn);
		MSG_PRINT(F(" Ver="));
		MSG_PRINT(ver);
		if (pn == 0 && ver > 0 && ver < 0xFF) {
			MSG_PRINTLN("");
			if (confmode) {
				Dstat = 0x1F;	// 'i'
			}
			else {
				Dstat = Dstat & 0x1F;
			}
		}
		else {
			MSG_PRINTLN(F(" invalid"));
			Dstat = ((Dstat & 0x0F) | 0x40);	// '-'  pn oder ver ungueltig
		}
	}
	else {
		MSG_PRINTLN(F(": timeout, no cc1101"));
		Dstat = ((Dstat & 0x0F) | 0xC0);		// '-'
	}
	return Dstat;
}
	
inline void configRadio()
{
	uint8_t remRadionr;	// Radionr merken
	
	if (cmdstring.charAt(3) >= 'A' && cmdstring.charAt(3) <= 'D') {
		remRadionr = radionr;
		radionr = (uint8_t)cmdstring.charAt(3) - 65;
		uint8_t cmd = cmdstring.charAt(2);
		if (cmd  != 'D') {
			uint8_t stat = tools::EEread(addr_statRadio + radionr);
			stat = radioDetekt(true, stat);
			if (cmd == 'E') {	// enable
				tools::EEwrite(addr_statRadio+radionr,stat);
				tools::EEstore();
				radio_bank[radionr] = stat;
				RXenabled[radionr] = true;
			}
		}
		else {	// disable
			tools::EEwrite(addr_statRadio+radionr,0xFF);
			tools::EEstore();
			radio_bank[radionr] = 0xFF;
			if (radionr == tools::EEread(addr_selRadio) && radionr != remRadionr) {	// wurde das im EEPROM gemerkte selektierte Radio deaktiviert und ist ungleich der aktuell selektierten
				tools::EEwrite(addr_selRadio, remRadionr);
				tools::EEstore();
			}
			if (radionr == remRadionr) {	// wurde das selektierte Radio deaktiviert?
				remRadionr = 4;
				radionr = 4;
				do {
					radionr--;
					if (radio_bank[radionr] < 10) {	// ist Radio aktiv, dann merken
						remRadionr = radionr;
					}
				} while (radionr > 0);
				if (remRadionr == 4) {	// wurde kein aktives Radio gefunden?
					remRadionr = defSelRadio;
				}
				tools::EEwrite(addr_selRadio, remRadionr);
				tools::EEstore();
				bank = radio_bank[remRadionr];
				bankOffset = getBankOffset(bank);
				//ccN = tools::EEbankRead(addr_ccN);
				ccmode = tools::EEbankRead(addr_ccmode);
			}
			CmdOk = true;
		}
		radionr = remRadionr;
	}
	else {
		unsuppCmd = true;
	}
	
}


#ifdef LAN_WIZ
void ethernetLoop()
{
	//check if there are any new clients
	if (server.available()) {
		if (!client || !client.connected()) {
			if (client) client.stop();
			client = server.available();
			MSG_PRINT(F("New client:"));
			MSG_PRINTLN(client.remoteIP());
			return;
		}
	}
}
#endif

void serialEvent()
{
  while (MSG_PRINTER.available())
  {
    char inChar = (char)MSG_PRINTER.read(); 
    switch(inChar)
    {
    case '\n':
    case '\r':
    case '\0':
    case '#':
		command_available=true;
		break;
    default:
      cmdstring += inChar;
    }
    if (cmdstring.length() > maxCmdString)
    {
	cmdstring = "";				// todo die restlichen Zeichen ignorieren
	MSG_PRINT(F("cmd to long! (max "));
	MSG_PRINT(maxCmdString);
	MSG_PRINTLN(F(")"));
    }
  }
}


int freeRam () {
#ifdef CMP_MEMDBG

 check_mem();

 MSG_PRINT("\nheapptr=[0x"); MSG_PRINT( (int) heapptr, HEX); MSG_PRINT("] (growing upward, "); MSG_PRINT( (int) heapptr, DEC); MSG_PRINT(" decimal)");

 MSG_PRINT("\nstackptr=[0x"); MSG_PRINT( (int) stackptr, HEX); MSG_PRINT("] (growing downward, "); MSG_PRINT( (int) stackptr, DEC); MSG_PRINT(" decimal)");

 MSG_PRINT("\ndifference should be positive: diff=stackptr-heapptr, diff=[0x");
 diff=stackptr-heapptr;
 MSG_PRINT( (int) diff, HEX); MSG_PRINT("] (which is ["); MSG_PRINT( (int) diff, DEC); MSG_PRINT("] (bytes decimal)");


 MSG_PRINT("\n\nLOOP END: get_free_memory() reports [");
 MSG_PRINT( get_free_memory() );
 MSG_PRINT("] (bytes) which must be > 0 for no heap/stack collision");


 // ---------------- Print memory profile -----------------
 MSG_PRINT("\n\n__data_start=[0x"); MSG_PRINT( (int) &__data_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__data_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__data_end=[0x"); MSG_PRINT((int) &__data_end, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__data_end, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__bss_start=[0x"); MSG_PRINT((int) & __bss_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__bss_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__bss_end=[0x"); MSG_PRINT( (int) &__bss_end, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__bss_end, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__heap_start=[0x"); MSG_PRINT( (int) &__heap_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__heap_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__malloc_heap_start=[0x"); MSG_PRINT( (int) __malloc_heap_start, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) __malloc_heap_start, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__malloc_margin=[0x"); MSG_PRINT( (int) &__malloc_margin, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) &__malloc_margin, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\n__brkval=[0x"); MSG_PRINT( (int) __brkval, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) __brkval, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\nSP=[0x"); MSG_PRINT( (int) SP, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) SP, DEC); MSG_PRINT("] bytes decimal");

 MSG_PRINT("\nRAMEND=[0x"); MSG_PRINT( (int) RAMEND, HEX ); MSG_PRINT("] which is ["); MSG_PRINT( (int) RAMEND, DEC); MSG_PRINT("] bytes decimal");

 // summaries:
 ramSize   = (int) RAMEND       - (int) &__data_start;
 dataSize  = (int) &__data_end  - (int) &__data_start;
 bssSize   = (int) &__bss_end   - (int) &__bss_start;
 heapSize  = (int) __brkval     - (int) &__heap_start;
 stackSize = (int) RAMEND       - (int) SP;
 freeMem1  = (int) SP           - (int) __brkval;
 freeMem2  = ramSize - stackSize - heapSize - bssSize - dataSize;
 MSG_PRINT("\n--- section size summaries ---");
 MSG_PRINT("\nram   size=["); MSG_PRINT( ramSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\n.data size=["); MSG_PRINT( dataSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\n.bss  size=["); MSG_PRINT( bssSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nheap  size=["); MSG_PRINT( heapSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nstack size=["); MSG_PRINT( stackSize, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nfree size1=["); MSG_PRINT( freeMem1, DEC ); MSG_PRINT("] bytes decimal");
 MSG_PRINT("\nfree size2=["); MSG_PRINT( freeMem2, DEC ); MSG_PRINT("] bytes decimal");
#else
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#endif // CMP_MEMDBG

 }

inline unsigned long getUptime()
{
	unsigned long now = millis();
	static uint8_t times_rolled = 0;
	static unsigned long last = 0;
	// If this run is less than the last the counter rolled
	if (now < last) {
		times_rolled++;
	}
	last = now;
	return (0xFFFFFFFF / 1000) * times_rolled + (now / 1000);
}

inline void getPing()
{
	MSG_PRINTLN(F("OK"));
	delayMicroseconds(500);
}

void en_dis_receiver(bool en_receiver)
{
	if (ccmode >= 15) {
		return;
	}
	if (en_receiver) {
		RXenabled[radionr] = true;
		enableReceive();
		MSG_PRINT(F("rx"));
		MSG_WRITE('A' + radionr);
		MSG_PRINT(F("=1"));
	}
	else {
		disableReceive();
		RXenabled[radionr] = false;
		MSG_PRINT(F("rx"));
		MSG_WRITE('A' + radionr);
		MSG_PRINT(F("=0"));
	}
	MSG_PRINT(F(" "));
}

inline void changeReceiver()
{
	if (cmdstring.charAt(1) != 'Q' && cmdstring.charAt(1) != 'E') {
		unsuppCmd = true;
		return;
	}

	uint8_t remRadionr = radionr;
	uint8_t remccmode = ccmode;
	uint16_t remBankOffset = bankOffset;
	uint8_t ra;
	uint8_t re;
	radionr = 255;
	if (cmdstring.charAt(2) >= 'A' && cmdstring.charAt(2) <= 'D') {	// Radio A-D
		radionr = (uint8_t)cmdstring.charAt(2) - 65;
		ra = radionr;
		re = radionr+1;
	}
	else {	// alle radio
		ra = 0;
		re = 4;
	}
	for (radionr = ra; radionr < re; radionr++) {
		if (radio_bank[radionr] > 9) {	// radio nicht aktiv
			continue;
		}
		bankOffset = getBankOffset(radio_bank[radionr]);
		ccmode = tools::EEbankRead(addr_ccmode);
		if (cmdstring.charAt(1) == 'E') {
			en_dis_receiver(true);
		}
		else {
			en_dis_receiver(false);
		}
	}
	MSG_PRINTLN("");
	
	radionr = remRadionr;
	ccmode = remccmode;
	bankOffset = remBankOffset;
}

void setCCmode() {
  if (ccmode == 0) {	// normal OOK
    enableReceive();
    if (radionr == 1) {
      pinAsOutput(PIN_SEND);
    }
  }
  else {		// mit ccFIFO
    if (radionr == 1) {
       pinAsInput(PIN_SEND);
    }
    disableReceive();
    if (cc1101::flushrx()) {
      enableReceive();
    }
  }
}

  void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
    if (hex < 16) {
      MSG_PRINT("0");
    }
    MSG_PRINT(hex, HEX);
  }


//================================= EEProm commands ======================================

void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t deb, int8_t led, int8_t mseq, int8_t tgBank)
{
	mu=mu<<1;
	mc=mc<<2;
	red=red<<3;
	deb=deb<<4;
	led=led<<5;
	mseq=mseq<<6;
	tgBank=tgBank<<7;
	int8_t dat =  ms | mu | mc | red | deb | led | mseq | tgBank;
	tools::EEwrite(addr_features,dat);
	tools::EEstore();
}

void callGetFunctions(void)
{
	 getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled, &musterDec.MredEnabled, &musterDec.MdebEnabled, &LEDenabled, &musterDec.MSeqEnabled, &toggleBankEnabled);
}

void getFunctions(bool *ms,bool *mu,bool *mc, bool *red, bool *deb, bool *led, bool *mseq, bool *tgBank)
{
    uint8_t high;
    uint8_t val;
    int8_t dat = tools::EEread(addr_features);

    *ms=bool (dat &(1<<0));
    *mu=bool (dat &(1<<1));
    *mc=bool (dat &(1<<2));
    *red=bool (dat &(1<<3));
    *deb=bool (dat &(1<<4));
    *led=bool (dat &(1<<5));
    *mseq=bool (dat &(1<<6));
    *tgBank=bool (dat &(1<<7));
    
    MdebFifoLimit = tools::EEread(CSetAddr[0]);
    musterDec.MsMoveCountmax = tools::EEread(CSetAddr[2]);
    val = tools::EEread(CSetAddr[3]);
    if (val == 0) {
       val = 1;
    }
    musterDec.maxMuPrint = val * 256;
    val = tools::EEread(CSetAddr[4]);
    if (val <=1 ) {
       musterDec.maxMsgSize = 254;
    }
    else if (val * 256 > defMaxMsgSize) {
       musterDec.maxMsgSize = defMaxMsgSize;
    }
    else {
       musterDec.maxMsgSize = val * 256;
    }
    musterDec.cMaxNumPattern = tools::EEread(CSetAddr[5]);

    high = tools::EEread(CSetAddr[CSet16]);
    musterDec.MuSplitThresh = tools::EEread(CSetAddr[CSet16+1]) + ((high << 8) & 0xFF00);
    high = tools::EEread(CSetAddr[CSet16+2]);
    musterDec.cMaxPulse = tools::EEread(CSetAddr[CSet16+3]) + ((high << 8) & 0xFF00);
    if (musterDec.cMaxPulse == 0) {
       musterDec.cMaxPulse = maxPulse;
    }
    musterDec.cMaxPulse = -musterDec.cMaxPulse;
    musterDec.mcMinBitLen = tools::EEread(CSetAddr[1]);
    if (musterDec.mcMinBitLen == 0) {
        musterDec.mcMinBitLen = mcMinBitLenDef;
    }
}

void getSelRadioBank(void)
{
    //radionr = defSelRadio;
    radionr = tools::EEread(addr_selRadio);
    if (radionr > 3) {
      radionr = defSelRadio;
    }
    bank = radio_bank[radionr];
    if (bank > 9) {
      bank = 0;
    }
    bankOffset = getBankOffset(bank);
    //ccN = tools::EEbankRead(addr_ccN);
    ccmode = tools::EEbankRead(addr_ccmode);
    if (ccmode == 255) {
       ccmode = 0;
       tools::EEbankWrite(addr_ccmode, ccmode);
       tools::EEstore();
    }
    ccmode = ccmode & 0x0F;
}


void getEthernetConfig(void)
{
	uint8_t i;
	
	if (tools::EEread(EE_MAC_ADDR) != mac_def[0] || tools::EEread(EE_MAC_ADDR+1) != mac_def[1] || tools::EEread(EE_MAC_ADDR+2) != mac_def[2]) {
		initEthernetConfig();
	}
	
	for (i = 0; i < 6; i++) {
		mac[i] = tools::EEread(EE_MAC_ADDR+i);
	}
	for (i = 0; i < 4; i++) {
		ip[i] = tools::EEread(EE_IP4_ADDR+i);
		gateway[i] = tools::EEread(EE_IP4_GATEWAY+i);
		netmask[i] = tools::EEread(EE_IP4_NETMASK+i);
	}
}

void initEthernetConfig(void)
{
	for (uint8_t i = 0; i < 4; i++) {
		tools::EEwrite(EE_IP4_ADDR+i,ip_def[i]);
		tools::EEwrite(EE_IP4_GATEWAY+i,gateway_def[i]);
		tools::EEwrite(EE_IP4_NETMASK+i,netmask_def[i]);
	}
	
	tools::EEwrite(EE_MAC_ADDR, mac_def[0]);
	tools::EEwrite(EE_MAC_ADDR+1, mac_def[1]);
	tools::EEwrite(EE_MAC_ADDR+2, mac_def[2]);
	uint32_t fserial = tools::flash_serial();
	tools::EEwrite(EE_MAC_ADDR+3, (uint8_t)(fserial>>16 & 0xff));
	tools::EEwrite(EE_MAC_ADDR+4, (uint8_t)(fserial>>8 & 0xff));
	tools::EEwrite(EE_MAC_ADDR+5, (uint8_t)(fserial & 0xff));
	tools::EEstore();
}

void initEEPROMconfig(void)
{
	tools::EEwrite(addr_features, 0x37);    	// Init EEPROM with all flags enabled, except red, nn and toggleBank
	
	for (uint8_t i = 0; i < CSetAnzEE; i++) {
		tools::EEwrite(CSetAddr[i], CSetDef[i]);
	}
	tools::EEwrite(addr_statRadio, defStatRadio);	// A
	tools::EEwrite(addr_statRadio+1, 0);    // Bank 0  B
	tools::EEwrite(addr_statRadio+2, defStatRadio); // C
	tools::EEwrite(addr_statRadio+3, defStatRadio); // D
	radio_bank[0] = defStatRadio;
	radio_bank[1] = 0;
	radio_bank[2] = defStatRadio;
	radio_bank[3] = defStatRadio;
	tools::EEwrite(addr_selRadio, defSelRadio);
	//tools::EEwrite(addr_bank, 0);
	tools::EEstore();
	MSG_PRINTLN(F("Init eeprom to defaults"));
}

void initEEPROM(void)
{
  if (tools::EEread(EE_MAGIC_OFFSET) == VERSION_1 && tools::EEread(EE_MAGIC_OFFSET+1) == VERSION_2) {
    
  //if (musterDec.MdebEnabled) {
    #ifdef DEBUG
    MSG_PRINTLN(F("Reading values from eeprom"));
    #endif
  //}

  } else {
    initEEPROMconfig();
    //storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled
    #ifdef CMP_CC1101
       if (tools::EEread(EE_MAGIC_OFFSET) != VERSION_1) {  // ccFactoryReset nur wenn VERSION_1 nicht passt
          cc1101::ccFactoryReset();	// nur Bank 0
       }
    #endif
    tools::EEwrite(EE_MAGIC_OFFSET, VERSION_1);
    tools::EEwrite(EE_MAGIC_OFFSET+1, VERSION_2);
    tools::EEstore();
  }
  callGetFunctions();
}
//-------------------------------------------------------------------------------------------------------------------
// <eof>
//-------------------------------------------------------------------------------------------------------------------