/*
*   RF_RECEIVER v3.35 for Arduino
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

	Version from: https://github.com/Ralf9/SIGNALDuino/tree/dev-r335_cc1101

*/

#include "compile_config.h"

#define PROGNAME               "RF_RECEIVER"
#define PROGVERS               "3.3.5-dev210522"
#define VERSION_1               0x33
#define VERSION_2               0x40

#ifdef CMP_CC1101
	#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
		#define PIN_LED               13
		#define PIN_SEND              9   // gdo0Pin TX out
		#define PIN_RECEIVE				   7
		#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
		#define PIN_MARK433			  4
		#define SS					  8  
	#elif ARDUINO_BUSWARE_CUL  // 8MHz
		#define PIN_LED                  7 // LED_BUILTIN
		#define PIN_SEND                 1 // gdo0Pin TX out
		#define PIN_RECEIVE              0 // gdo2
		#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
		#define PIN_MARK433              10
		#define SS                       17
		#include <avr/power.h>
	#elif ARDUINO_ATMEGA328P_MINICUL  // 8MHz
		#define PIN_LED               4
		#define PIN_SEND              2   // gdo0Pin TX out
		#define PIN_RECEIVE           3
		#define PIN_MARK433	      A0
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


#define BAUDRATE               57600
#define FIFO_LENGTH            140 // 50

#define DEBUG                  1

#ifdef WATCHDOG
	#include <avr/wdt.h>
#endif
#include "FastDelegate.h"
#include "output.h"
#include "bitstore.h"
#include "signalDecoder.h"
#include <TimerOne.h>  // Timer for LED Blinking

#include "SimpleFIFO.h"
SimpleFIFO<int,FIFO_LENGTH> FiFo; //store FIFO_LENGTH # ints
SignalDetectorClass musterDec;


#include <EEPROM.h>
#include "cc1101.h"

#define pulseMin  90
#define maxCmdString 350  //350 // 250
#define maxSendPattern 8
#define mcMinBitLenDef   17
//#define ccMaxBuf 50
#define maxSendEcho 100

// EEProm Address
#define EE_MAGIC_OFFSET      0
//#define addr_togglesec       0x3C
#define addr_ccN             0x3D
#define addr_ccmode          0x3E
//#define addr_featuresB       0x3F
#define addr_bankdescr       0x40    // 0x40-0x47 bis Bank 9 0x88-0x8F  # Bank 0 bis Bank 9, Kurzbeschreibungen (max 8 Zeichen)
#define addr_bank            0xFD
#define addr_features        0xFF

volatile bool blinkLED = false;
String cmdstring = "";
char msg_cmd0 = ' ';
char msg_cmd1 = ' ';
volatile unsigned long lastTime = micros();
bool hasCC1101 = false;
bool LEDenabled = true;
bool toggleBankEnabled = false;
bool RXenabled = true;			// 1 - enable receive, Zwischenspeicher zum enablereceive merken
bool unsuppCmd = false;
uint8_t MdebFifoLimit = 120;
uint8_t bank = 0;
uint16_t bankOffset = 0;
//uint8_t ccN = 0;
uint8_t ccmode = 0;		// cc1101 Mode: 0 - normal, 1 - FIFO, 2 - FIFO ohne dup, 3 - FIFO LaCrosse, 4 - FIFO LaCrosse, 9 - FIFO mit Debug Ausgaben
uint8_t ccBuf[ccMaxBuf+2];

void cmd_help_S();
void cmd_help();
void cmd_bank();
void configCMD();
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
void cmd_test();
void cmd_Version();
void cmd_writeEEPROM();
void cmd_writePatable();
void changeReceiver();

//typedef void (* GenericFP)(int); //function pointer prototype to a function which takes an 'int' an returns 'void'
#define cmdAnz 22
const char cmd0[] =  {'?', '?', 'b', 'C', 'C', 'C', 'C', 'C', 'C', 'e', 'e', 'P', 'r', 'R', 'S', 't', 'T', 'V', 'W', 'x', 'X', 'X'};
const char cmd1[] =  {'S', ' ', ' ', 'E', 'D', 'G', 'S', 'W', ' ', 'C', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'E', 'Q'};
const bool cmdCC[] = {  0,   0,   0,   0,   0,   0,   0,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,  0 };
void (*cmdFP[])(void) = {
		cmd_help_S, // ?S
		cmd_help,	// ?
		cmd_bank,	// b
		configCMD,	// CE
		configCMD,	// CD
		getConfig,	// CG
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
		cmd_test, // T
		cmd_Version,	// V
		cmd_writeEEPROM,// W
		cmd_writePatable,// x
		changeReceiver,	// XE
		changeReceiver	// XQ
		};

#define CSetAnz 9
#define CSetAnzEE 11
#define CSet16 7
#define CSccN 5
#define CSccmode 6

//const char *CSetCmd[] = {"fifolimit", "mcmbl", "mscnt", "muoverflmax", "maxnumpat", "ccN",    "ccmode", "muthresh", "L",  "maxpulse", "L"  };
const uint8_t CSetAddr[] = {  0xf0,     0xf1,    0xf2,    0xf3,          0xf4,     addr_ccN, addr_ccmode,       0xf5,  0xf6, 0xf7,      0xf8 };
const uint8_t CSetDef[] =  {    120,       0,       4,       3,             8,            0,           0,          0,     0,    0,         0 };

const char string_0[] PROGMEM = "fifolimit";
const char string_1[] PROGMEM = "mcmbl";
const char string_2[] PROGMEM = "mscnt";
const char string_3[] PROGMEM = "muoverflmax";
const char string_4[] PROGMEM = "maxnumpat";
const char string_5[] PROGMEM = "ccN";
const char string_6[] PROGMEM = "ccmode";
const char string_7[] PROGMEM = "muthresh";
const char string_8[] PROGMEM = "maxpulse";

const char * const CSetCmd[] PROGMEM = { string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8};

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
uint8_t cmdstringPos2int(uint8_t pos);
void printHex2(const byte hex);
uint8_t rssiCallback() { return 0; };	// Dummy return if no rssi value can be retrieved from receiver


void setup() {
#if defined(ARDUINO_BUSWARE_CUL)
	clock_prescale_set(clock_div_1);
#endif
	uint8_t ccVersion;
	Serial.begin(BAUDRATE);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
	}
	//if (musterDec.MdebEnabled) {
	DBG_PRINTLN(F("Using sFIFO"));
	//}
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
	MSG_PRINT(F("CCInit "));	
	cc1101::CCinit();				// CC1101 init
	ccVersion = cc1101::getCCVersion();
	if (ccVersion == 0x00 || ccVersion == 0xFF)	// checks if valid Chip ID is found. Usualy 0x03 or 0x14.
	{
		MSG_PRINT(F("no CC11xx found!"));
		hasCC1101 = false;
	}
	else {
		MSG_PRINT(F("ok."));
		hasCC1101 = true;
	}
	MSG_PRINT(F(" Ver=0x"));
	printHex2(ccVersion);
	MSG_PRINT(F(" Partn="));
	MSG_PRINTLN(cc1101::getCCPartnum());
	
	if (hasCC1101)
	{
		musterDec.setRSSICallback(&cc1101::getRSSI);                    // Provide the RSSI Callback
	} 
	else
		musterDec.setRSSICallback(&rssiCallback);	// Provide the RSSI Callback		
#endif 

	if (ccmode == 0) {
		pinAsOutput(PIN_SEND);
	}
	if (musterDec.MdebEnabled) {
		MSG_PRINTLN(F("Starting timerjob"));
	}
	delay(50);

	Timer1.initialize(31*1000); //Interrupt wird jede 31 Millisekunden ausgeloest
	Timer1.attachInterrupt(cronjob);

	cmdstring.reserve(maxCmdString);

#ifdef CMP_CC1101
  if (hasCC1101) {
	if (ccmode > 0 || cc1101::regCheck()) {
#endif
#ifndef SENDTODECODER
		enableReceive();
		if (musterDec.MdebEnabled) {
			MSG_PRINTLN(F("receiver enabled"));
		}
#endif
#ifdef CMP_CC1101
	}
	else {
		MSG_PRINTLN(F("cc1101 is for OOK not correctly set."));
	}
  }
#endif
}

void cronjob() {
	static uint16_t cnt0 = 0;
	static uint8_t cnt1 = 0;
	cli();
	 const unsigned long  duration = micros() - lastTime;
	 
	 if (duration > maxPulse && ccmode == 0) { //Auf Maximalwert pruefen.
		 int sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE)) { // Wenn jetzt low ist, ist auch weiterhin low
			 sDuration = -sDuration;
		 }
		 FiFo.enqueue(sDuration);

		 lastTime = micros();
	
	 }
	sei();
	
	digitalWrite(PIN_LED, blinkLED);
	blinkLED = false;
	if (cnt0++ == 0) {
		if (cnt1++ == 0) {
			getUptime();
		}
	}
}


void loop() {
	static int aktVal=0;
	bool state;
	uint8_t fifoCount;
	
#ifdef __AVR_ATmega32U4__	
	serialEvent();
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
#ifndef ONLY_FSK
  if (ccmode == 0) {
	musterDec.printMsgSuccess = false;
	musterDec.NoMsgEnd = false;
	while (FiFo.count()>0 ) { //Puffer auslesen und an Dekoder uebergeben

		aktVal=FiFo.dequeue();
		state = musterDec.decode(&aktVal);
		if (musterDec.MdebEnabled && musterDec.printMsgSuccess) {
			fifoCount = FiFo.count();
			if (fifoCount > MdebFifoLimit) {
				if (musterDec.NoMsgEnd) {
					if (musterDec.MredEnabled) {
						MSG_PRINT(F("F"));
						MSG_PRINT(fifoCount, HEX);
						MSG_PRINT(F(";"));
					}
				}
				else {
					MSG_PRINT(F("MF="));
					MSG_PRINTLN(fifoCount, DEC);
				}
			}
		}
		if (musterDec.printMsgSuccess && LEDenabled) {
			blinkLED=true; //LED blinken, wenn Meldung dekodiert
		}
		musterDec.printMsgSuccess = false;
	}
  }
  else {
#else
  if (ccmode > 0) {
#endif
	uint8_t fifoBytes;
	bool dup;		// true bei identischen Wiederholungen bei readRXFIFO

	if (isHigh(PIN_RECEIVE)) {  // wait for CC1100_FIFOTHR given bytes to arrive in FIFO
		if (LEDenabled) {
			blinkLED=true;
			
		}
		if (ccmode == 4) {
			cc1101::ccStrobe_SIDLE();	// start over syncing
		}
		
		fifoBytes = cc1101::getRXBYTES(); // & 0x7f; // read len, transfer RX fifo
		if (fifoBytes > 0) {
			uint8_t marcstate;
			bool appendRSSI = false;
			uint8_t RSSI;
			if ((EEPROM.read(bankOffset + 2 +CC1101_PKTCTRL1) & 4) == 4) {
				appendRSSI = true;
			}
			else {
				RSSI = cc1101::getRSSI();
			}
			
			if (ccmode == 9) {
				MSG_PRINT(F("RX("));
				MSG_PRINT(fifoBytes);
				MSG_PRINT(F(") "));
			}
			if (fifoBytes < 0x80) {	// RXoverflow?
				if (fifoBytes > ccMaxBuf) {
					fifoBytes = ccMaxBuf;
				}
				dup = cc1101::readRXFIFO(fifoBytes, ccmode, appendRSSI);
				if (ccmode != 2 || dup == false) {
					if (ccmode != 9) {
						MSG_PRINT(MSG_START);
						MSG_PRINT(F("MN;D="));
					}
					for (uint8_t i = 0; i < fifoBytes; i++) {
						printHex2(ccBuf[i]);
						//MSG_PRINT(" ");
					}
					if (ccmode == 9) {
						MSG_PRINT(F(" ("));
						MSG_PRINT(cc1101::getRXBYTES());
						MSG_PRINT(F(")"));
					}
					else {
						uint8_t n = EEPROM.read(bankOffset + addr_ccN);
						if (n > 0) {
							MSG_PRINT(F(";N="));
							MSG_PRINT(n);
						}
						if (appendRSSI == true) {
							MSG_PRINT(F(";r"));
						}
						else {
							MSG_PRINT(F(";R="));
							MSG_PRINT(RSSI);
						}
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
				else if (marcstate != 13 && ccmode < 3) {  // marcstate 13 ist rx
					cc1101::setReceiveMode();
				}
			}
		}
	  }
	}
 }



//========================= Pulseauswertung ================================================
void handleInterrupt() {
  cli();
  const unsigned long Time=micros();
  //const bool state = digitalRead(PIN_RECEIVE);
  const unsigned long  duration = Time - lastTime;
  lastTime = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
	int sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
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
  sei();
}

void enableReceive() {
  if (RXenabled == true) {
   #ifndef ONLY_FSK
   if (ccmode == 0) {	// normal
     attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), handleInterrupt, CHANGE);
   }
   #endif
   #ifdef CMP_CC1101
   if (hasCC1101) {
     cc1101::ccStrobe_SIDLE();	// Idle mode
     delay(1);
     cc1101::setReceiveMode();
   }
   #endif
  }
}

void disableReceive() {
  detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE));

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

	for (uint16_t i=startpos;i<=endpos;i++ )
	{
		//MSG_PRINT(cmdstring.substring(i,i+1));
		index = source->charAt(i) - '0';
		//MSG_PRINT(index);
		isLow=buckets[index] >> 15;
		dur = abs(buckets[index]); 		//isLow ? dur = abs(buckets[index]) : dur = abs(buckets[index]);
#ifndef SENDTODECODER
		while (stoptime > micros()){
			;
		}
		isLow ? digitalLow(PIN_SEND): digitalHigh(PIN_SEND);
		stoptime+=dur;
#else
		if (isLow) dur = -dur;
		musterDec.decode(&dur);
#endif
	}
#ifndef SENDTODECODER
	while (stoptime > micros()){
		;
	}
#endif
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
			MSG_PRINT(F("D="));
			MSG_PRINTLN(cmdstring.substring(startdata, start_pos-1));
#endif
		} else if (msg_cmd0 == 'C') {
			command[cmdNo].sendclock = cmdstring.substring(startdata, start_pos-1).toInt();
			extraDelay = true;
#ifdef DEBUGSENDCMD
			MSG_PRINT(F("C="));
			MSG_PRINTLN(command[cmdNo].sendclock);
#endif
		} else if (msg_cmd0 == 'F') {
			ccParamAnz = (start_pos - startdata) / 2;
#ifdef DEBUGSENDCMD
			MSG_PRINT(F("F="));
#endif
#ifndef SENDTODECODER
			if (ccParamAnz > 0 && ccParamAnz <= 5 && hasCC1101) {
				uint8_t hex;
				//MSG_PRINTLN("write new ccreg  ");
				for (uint8_t i=0;i<ccParamAnz;i++)
				{
					hex = (uint8_t)cmdstring.charAt(startdata + i*2);
					val = cc1101::hex2int(hex) * 16;
					hex = (uint8_t)cmdstring.charAt(startdata+1 + i*2);
					val = cc1101::hex2int(hex) + val;
					cc1101::writeReg(0x0d + i, val);            // neue Registerwerte schreiben
  #ifdef DEBUGSENDCMD
					printHex2(val);
  #endif
				}
			}
#endif
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
#ifndef SENDTODECODER
		#ifdef CMP_CC1101
		if (hasCC1101) cc1101::setTransmitMode();	
		#endif
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

#ifndef SENDTODECODER
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
				val = EEPROM.read(0x0f + i);
				printHex2(val);
				cc1101::writeReg(0x0d + i, val);    // gemerkte Registerwerte zurueckschreiben
			}
			//MSG_PRINTLN("");
		}
#endif
	}
#ifndef SENDTODECODER
	enableReceive();	// enable the receiver
	MSG_PRINTLN("");
#endif
}


void send_ccFIFO()
{
	uint8_t repeats=1;  // Default is always one iteration so repeat is 1 if not set
	int8_t startpos=0;
	int8_t endpos=0;
	int8_t startn=0;
	int8_t sendN=0;
	uint8_t startdata=0;
	uint8_t enddata=0;
	
	startpos = cmdstring.indexOf(";R=",2);
	
	startn = cmdstring.indexOf(";N=",2);
	
	startdata = cmdstring.indexOf(";D=",2);
	if (startdata > 0) {
		if (startn > 0) {	// N= gefunden
			endpos = startn;
			sendN = cmdstring.substring(startn+3, startdata).toInt();
		}
		else {
			endpos = startdata;
		}
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
				cc1101::setTransmitMode();
				cc1101::sendFIFO(startdata, enddata);
			}
			MSG_PRINT(cmdstring); // echo
			MSG_PRINT(F("Marcs="));
			uint8_t marcstate = cc1101::getMARCSTATE();
			MSG_PRINTLN(marcstate);  // 13 - rx
			if (marcstate != 13 && RXenabled == true) {
				if (marcstate != 1) {          // not idle
					cc1101::ccStrobe_SIDLE();  // goto Idle mode
					delay(1);
				}
				cc1101::setReceiveMode();
			}
		}
		else {
			startdata = -1;
		}
	}
	if (startdata == -1)
	{
		MSG_PRINTLN(F("send failed!"));
	}
}

//================================= Kommandos ======================================
void IT_CMDs();

void HandleCommand()
{
	uint8_t reg;
	uint8_t val;
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
	if (i < cmdAnz) {
		//MSG_PRINT(F(" "));
		//MSG_PRINT(cmd0[i]);
		//MSG_PRINT(cmd1[i]);
		cmdFP[i]();
	}
	//MSG_PRINTLN("");
	
	if (unsuppCmd) {
		MSG_PRINTLN(F("Unsupported command"));
	}
}


void cmd_help_S()	// get help configvariables
{
	char buffer[12];
	for (uint8_t i = 0; i < CSetAnz; i++) {
	    strcpy_P(buffer, (char*)pgm_read_word(&(CSetCmd[i])));
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
	if (isDigit(cmdstring.charAt(1))) {
		uint8_t digit;
		uint8_t bankOld = bank;
		uint16_t bankOffsetOld = bankOffset;
		digit = (uint8_t)cmdstring.charAt(1);
		bank = cc1101::hex2int(digit);
		bankOffset = getBankOffset(bank);
		if (cmdstring.charAt(2) == '-' && bank > 0 && bank != bankOld) {  // Bank deaktivieren (ungueltig)
			EEPROM.write(bankOffset, 255);
			EEPROM.write(bankOffset+1, 255);
			MSG_PRINT(F("Bank "));
			MSG_PRINT(bank);
			MSG_PRINTLN(F(" clear"));
			bank = bankOld;
			bankOffset = bankOffsetOld;
			return;
		}
		if (bank == 0 || cmdstring.charAt(0) == 'e' || (EEPROM.read(bankOffset) == bank && EEPROM.read(bankOffset + 1) == (255 - bank))) {
		  if (cmdstring.charAt(2) == 'f') {
			uint8_t ccmodeOld = ccmode;
			ccmode = EEPROM.read(bankOffset + addr_ccmode);
			if (ccmodeOld > 0 && ccmode > 0) {
				cc1101::ccStrobe_SIDLE();	// Idle mode
				uint8_t val;
				uint8_t n = 0;
				for (uint8_t i = 0; i <= 0x26; i++) {
					val = EEPROM.read(bankOffset + 2 + i);
					if (val != EEPROM.read(bankOffsetOld + 2 + i)) {
						n++;
						//printHex2(i);
						//printHex2(val);
						//MSG_PRINT(F(" "));
						cc1101::writeReg(i,val);
					}
				}
				//MSG_PRINTLN("");
				cc1101::flushrx();
				cc1101::setReceiveMode();
				MSG_PRINT("fn=");
				MSG_PRINT(n);
				MSG_PRINT(" ");
				print_Bank();
			}
			else {
				bankOffset = bankOffsetOld;
				bank = bankOld;
				ccmode = ccmodeOld;
				MSG_PRINTLN(F("ccmode not valid"));
			}
		  }
		  else {
			if (cmdstring.charAt(2) == 'W') {
				EEPROM.write(addr_bank, bank);
				MSG_PRINT(F("write "));
			}
			MSG_PRINT(F("set "));
			//ccN = EEPROM.read(bankOffset + addr_ccN);
			ccmode = EEPROM.read(bankOffset + addr_ccmode);
			if (ccmode == 255) {
				ccmode = 0;
				EEPROM.write(bankOffset + addr_ccmode, ccmode);
			}
			if (cmdstring.charAt(0) != 'e') {
				print_Bank();
				cc1101::CCinit();
				setCCmode();
			}
		  }
		}
		else {
			MSG_PRINT(F("The Bank "));
			MSG_PRINT(bank);
			MSG_PRINT(F(" was not init, therefore it reseted to sduino defaults (raw e). "));
			cmd_ccFactoryReset();
		}
	}
	else if (cmdstring.charAt(0) == 'b' && (cmdstring.length() == 1 || cmdstring.charAt(1) == '?')) {
		print_Bank();
	}
	else if (cmdstring.charAt(1) == 's') {
		print_bank_sum();
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
	printHex2(EEPROM.read(bankOffs + 2 + CC1101_SYNC1));
	printHex2(EEPROM.read(bankOffs + 2 + CC1101_SYNC0));
	MSG_PRINT(F(" ccconf="));
	for (uint8_t i = 0x0F; i <= 0x1F; i++) {
		printHex2(EEPROM.read(bankOffs + i));
	}
	MSG_PRINT(F(" boffs="));
	sprintf(hexString, "%04X", bankOffs);
	MSG_PRINT(hexString);
}

void print_Bank()
{
	uint8_t tmp_ccN = EEPROM.read(bankOffset + addr_ccN);
	
	MSG_PRINT(F("b="));
	MSG_PRINT(bank);
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
		
		sBankoff = getBankOffset(i);
		if ((EEPROM.read(sBankoff) == i && EEPROM.read(sBankoff+1) == (255 - i)) || i == 0) {
			Nstr[i2] = '0' + EEPROM.read(sBankoff + addr_ccN);
			sCcmode = EEPROM.read(sBankoff + addr_ccmode);
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
	Nstr[20] = 0;
	ccmodeStr[20] = 0;
	
	MSG_PRINT(F("Bank__ "));
	MSG_PRINT(bankStr);
	MSG_PRINT(F(" Radio_ "));
	for (i = 0; i <= 9; i++) {
		i2 = i * 2;
		if (i != bank) {
			bankStr[i2] = '-';
		}
		else {
			bankStr[i2] = '*';
		}
		bankStr[i2+1] = ' ';
	}
	bankStr[20] = 0;
	MSG_PRINT(bankStr);
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
					ch = EEPROM.read(addr_bankdescr + (i * 8) + j);
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

void cmd_Version()	// V: Version
{
	MSG_PRINT(F("V " PROGVERS " SIGNALduino "));
	if (hasCC1101) {
	    MSG_PRINT(F("cc1101 "));
#ifdef ARDUINO_ATMEGA328P_MINICUL
	    MSG_PRINT(F("(minicul "));
#endif
#ifdef ARDUINO_BUSWARE_CUL
	    MSG_PRINT(F("(culV3 "));
#endif
#ifdef PIN_MARK433
	    MSG_PRINT(isLow(PIN_MARK433) ? "433" : "868");
	    MSG_PRINT(F("MHz) "));
#endif
	}
#ifdef SENDTODECODER
	MSG_PRINT(F("(no receive, only send to decoder) "));
#endif
#ifdef ONLY_FSK
    MSG_PRINT(F("only xFSK "));
#endif
	MSG_PRINT(F("(b"));
	if (toggleBankEnabled == false) {
		MSG_PRINT(bank);
	} else {
		MSG_PRINT(F("x"));
	}
	MSG_PRINT(F(") "));
	MSG_PRINTLN(F("- compiled at " __DATE__ " " __TIME__));
}

void cmd_freeRam()	// R: FreeMemory
{
	MSG_PRINTLN(freeRam());
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

void cmd_test()	// T<bank><sec>
{
    unsuppCmd = true;
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
	bool resetFlag = false;
    
	uint8_t CWccreset = EEPROM.read(bankOffset + addr_CWccreset);
	if ((CWccreset == 0xA5 || CWccreset == 0xA6) &&  cmdstring.charAt(6) == ',') { 
		cc1101::ccFactoryReset(false);
		cc1101::CCinit();
		resetFlag = true;
	}

	pos = 2;
	for (i=0; i<64; i++)
	{
		if (!isHexadecimalDigit(cmdstring.charAt(pos)) || !isHexadecimalDigit(cmdstring.charAt(pos+1)) || !isHexadecimalDigit(cmdstring.charAt(pos+2)) || !isHexadecimalDigit(cmdstring.charAt(pos+3))) {
			break;
		}
		reg = cmdstringPos2int(pos);
		if (reg > 0x47) {
			break;
		}
		val = cmdstringPos2int(pos+2);
		if (reg <= 0x2F) {
			cc1101::writeReg(reg,val);
			if (reg <= 0x28) {
				reg += 2;
			}
			else if (reg < 0x2C) {  // 0x29 - 0x2B nur fuer Testzwecke -> kein write ins EEPROM
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
				EEPROM.write(bankOffset + reg, val);
				if (reg == 0x37) {		// Ende der patable
					cc1101::writePatable();
				}
			}
		}
		else {		// 0x40 - 0x47  Bank Kurzbeschreibung (max 8 Zeichen)
			reg = reg + (bank * 8);
			EEPROM.write(reg, val);
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
		if (resetFlag == true) {
			MSG_PRINT(F(",e"));
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
		reg = cmdstringPos2int(1);
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
	
    if (cmdstring.charAt(1) == 'S' && cmdstring.charAt(2) == '3' && hasCC1101) {       // WS<reg>  Command Strobes
        cc1101::commandStrobes();
    } else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2)) && isHexadecimalDigit(cmdstring.charAt(3)) && isHexadecimalDigit(cmdstring.charAt(4))) {
         reg = cmdstringPos2int(1);
         val = cmdstringPos2int(3);
         if (reg < 0x40) {
           EEPROM.write(bankOffset + reg, val);
           if (hasCC1101 && reg <= 0x2A) {  // nur bis cc1101_reg 0x28
             cc1101::writeReg(reg-2, val);
           }
         }
         else {   // ab 0x40 immer in Bank 0
           EEPROM.write(reg, val);
         }
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
		
		high = cmdstringPos2int(2);
		low = cmdstringPos2int(4);
		addr = high * 256 + low;
		
		for (uint8_t j = 0; j < 4; j++) {
			sprintf(hexString, "%04X", addr);
			MSG_PRINT(F("EEPROM "));
			MSG_PRINT(hexString);
			MSG_PRINT(F(":"));
			for (uint8_t i = 0; i < 16; i++) {
				MSG_PRINT(F(" "));
				printHex2(EEPROM.read(addr + i));
			}
			addr += 16;
			MSG_PRINT(F("  "));
		}
		MSG_PRINTLN("");
	}
   else if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {             // r<adr>  read EEPROM
     uint8_t reg;
     reg = cmdstringPos2int(1);
     MSG_PRINT(F("EEPROM "));
     printHex2(reg);
     if (cmdstring.charAt(3) == 'n') {
         MSG_PRINT(F(" :"));
         for (uint8_t i = 0; i < 16; i++) {
             MSG_PRINT(F(" "));
             printHex2(EEPROM.read(bankOffset + reg + i));
         }
     } else {
        MSG_PRINT(F(" = "));
        printHex2(EEPROM.read(bankOffset + reg));
     }
     MSG_PRINTLN("");
  } else {
     unsuppCmd = true;
  }
}

void cmd_writePatable()
{
  uint8_t val;
  if (isHexadecimalDigit(cmdstring.charAt(1)) && isHexadecimalDigit(cmdstring.charAt(2))) {
     val = cmdstringPos2int(1);
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
         initEEPROMconfig();
         callGetFunctions();
         cc1101::CCinit();
         RXenabled = true;
         setCCmode();
}

void cmd_ccFactoryReset()
{
         if (cmdstring.charAt(0) == 'e' && isDigit(cmdstring.charAt(1))) {
            cmd_bank();
         }
         cc1101::ccFactoryReset(true);
         cc1101::CCinit();
         EEPROM.write(bankOffset + addr_ccN, 0);
         ccmode = 0;
         EEPROM.write(bankOffset + addr_ccmode, ccmode);
         setCCmode();
         if (bank > 0) {
            EEPROM.write(bankOffset, bank);
            EEPROM.write(bankOffset + 1, (255 - bank));
         }
         if (cmdstring.charAt(0) == 'e') {
            print_Bank();
         }
}


uint8_t cmdstringPos2int(uint8_t pos) {
  uint8_t val;
  uint8_t hex;
  
       hex = (uint8_t)cmdstring.charAt(pos);
       val = cc1101::hex2int(hex) * 16;
       hex = (uint8_t)cmdstring.charAt(pos+1);
       val = cc1101::hex2int(hex) + val;
       return val;
}

inline void getConfig()
{
  if (ccmode == 0) {
   MSG_PRINT(F("MS="));
   MSG_PRINT(musterDec.MSenabled,DEC);
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
    MSG_PRINT(F(" b="));
    MSG_PRINT(bank);
  }
   if (LEDenabled == false) {
      MSG_PRINT(F(";LED=0"));
   }
   if (RXenabled == false) {
      MSG_PRINT(F(";RX=0"));
   }
   /*if (toggleBankEnabled == true) {
      MSG_PRINT(F(";toggleBank=1"));
   }*/
  if (ccmode == 0) {
   if (musterDec.MuNoOverflow == true) {
      MSG_PRINT(F(";MuNoOverflow=1"));
   }
   MSG_PRINT(F(";Mdebug="));
   MSG_PRINT(musterDec.MdebEnabled, DEC);
   MSG_PRINT(F("_MScnt="));
   MSG_PRINT(musterDec.MsMoveCountmax, DEC);
   if (musterDec.MuNoOverflow == true) {
      MSG_PRINT(F(";MuOverflMax="));
      MSG_PRINT(musterDec.MuOverflMax, DEC);
   }
   else {
      MSG_PRINT(F(";MuSplitThresh="));
      MSG_PRINT(musterDec.MuSplitThresh, DEC);
   }
   if (musterDec.mcMinBitLen != mcMinBitLenDef) {
      MSG_PRINT(F(";mcMinBitLen="));
      MSG_PRINT(musterDec.mcMinBitLen, DEC);
   }
   if (musterDec.cMaxNumPattern != CSetDef[4]) {
      MSG_PRINT(F(";maxNumPat="));
      MSG_PRINT(musterDec.cMaxNumPattern, DEC);
   }
   if (musterDec.cMaxPulse != -maxPulse) {
      MSG_PRINT(F(";maxPulse="));
      MSG_PRINT(musterDec.cMaxPulse, DEC);
   }
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
  else if (cmdstring.charAt(2) == 'O') {  //
	bptr=&musterDec.MuNoOverflow;
  }
  else if (cmdstring.charAt(2) == 'T') {  // toggleBankEnabled
	bptr=&toggleBankEnabled;
  }

  if (cmdstring.charAt(1) == 'E') {   // Enable
	*bptr=true;
  }
  else if (cmdstring.charAt(1) == 'D') {  // Disable
	*bptr=false;
  } else {
	return;
  }
  storeFunctions(musterDec.MSenabled, musterDec.MUenabled, musterDec.MCenabled, musterDec.MredEnabled, musterDec.MdebEnabled, LEDenabled, musterDec.MuNoOverflow, toggleBankEnabled);
}

inline void configSET()
{ 
	char buffer[12];
	int16_t i = cmdstring.indexOf("=",4);
	uint8_t n = 0;
	uint8_t val;
	uint16_t val16;
	if (i < 0) {
		unsuppCmd = true;
	}
	while (n < CSetAnz) {
		strcpy_P(buffer, (char*)pgm_read_word(&(CSetCmd[n])));
		if (cmdstring.substring(2, i) == buffer) {
			MSG_PRINT(buffer);
			MSG_PRINT(F("="));
			if (n < CSet16) {
				val = cmdstring.substring(i+1).toInt();
				MSG_PRINTLN(val);
				if (n == CSccmode || n == CSccN) {
					EEPROM.write(bankOffset + CSetAddr[n], val);
				} else {
					EEPROM.write(CSetAddr[n], val);
				}
			}
			else {
				val16 = cmdstring.substring(i+1).toInt();
				MSG_PRINTLN(val16);
				val = (val16>>8) & 0xFF;
				EEPROM.write(CSetAddr[n+(n-CSet16)], val);		// high
				val = val16 & 0xFF;
				EEPROM.write(CSetAddr[n+(n-CSet16)+1], val);	// low
			}
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
	else if (n == 3) {			// MuOverflMax
		musterDec.MuOverflMax = val;
	}
	else if (n == 4) {			// maxnumpat
		musterDec.cMaxNumPattern = val;
	}
//	else if (n == 5) {			// ccN
//		ccN = val;
//	}
	else if (n == 6) {			// ccmode
		ccmode = val;
		setCCmode();
	}
	else if (n == 7) {			// muthresh
		musterDec.MuSplitThresh = val16;
	}
	else if (n == 8) {			// maxpulse
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

inline void changeReceiver() {
  if (cmdstring.charAt(1) == 'Q')
  {
  	disableReceive();
	RXenabled = false;
	MSG_PRINTLN("RX=0");
  }
  if (cmdstring.charAt(1) == 'E')
  {
#ifndef SENDTODECODER
	RXenabled = true;
  	enableReceive();
	MSG_PRINTLN("RX=1");
#endif
  }
}

void setCCmode() {
  if (ccmode == 0) {	// normal OOK
    enableReceive();
    pinAsOutput(PIN_SEND);
  }
  else {		// mit ccFIFO
    pinAsInput(PIN_SEND);
    disableReceive();
    cc1101::flushrx();
    enableReceive();
  }
}

  void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
    if (hex < 16) {
      MSG_PRINT("0");
    }
    MSG_PRINT(hex, HEX);
  }



//================================= EEProm commands ======================================

void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t deb, int8_t led, int8_t overfl, int8_t tgBank)
{
	mu=mu<<1;
	mc=mc<<2;
	red=red<<3;
	deb=deb<<4;
	led=led<<5;
	overfl=overfl<<6;
	tgBank=tgBank<<7;
	int8_t dat =  ms | mu | mc | red | deb | led | overfl | tgBank;
    EEPROM.write(addr_features,dat);
}

void callGetFunctions(void)
{
	 getFunctions(&musterDec.MSenabled, &musterDec.MUenabled, &musterDec.MCenabled, &musterDec.MredEnabled, &musterDec.MdebEnabled, &LEDenabled, &musterDec.MuNoOverflow, &toggleBankEnabled);
}

void getFunctions(bool *ms,bool *mu,bool *mc, bool *red, bool *deb, bool *led, bool *overfl, bool *tgBank)
{
    int8_t high;
    int8_t dat = EEPROM.read(addr_features);

    *ms=bool (dat &(1<<0));
    *mu=bool (dat &(1<<1));
    *mc=bool (dat &(1<<2));
    *red=bool (dat &(1<<3));
    *deb=bool (dat &(1<<4));
    *led=bool (dat &(1<<5));
    *overfl=bool (dat &(1<<6));
    *tgBank= bool (dat &(1<<7));
    
    MdebFifoLimit = EEPROM.read(CSetAddr[0]);
    musterDec.MsMoveCountmax = EEPROM.read(CSetAddr[2]);
    musterDec.MuOverflMax = EEPROM.read(CSetAddr[3]);
    musterDec.cMaxNumPattern = EEPROM.read(CSetAddr[4]);
    bank = EEPROM.read(addr_bank);
    if (bank > 9) {
      bank = 0;
    }
    bankOffset = getBankOffset(bank);
    //ccN = EEPROM.read(bankOffset + addr_ccN);
    ccmode = EEPROM.read(bankOffset + addr_ccmode);
    if (ccmode == 255) {
       ccmode = 0;
       EEPROM.write(bankOffset + addr_ccmode, ccmode);
    }
    high = EEPROM.read(CSetAddr[CSet16]);
    musterDec.MuSplitThresh = EEPROM.read(CSetAddr[CSet16+1]) + ((high << 8) & 0xFF00);
    high = EEPROM.read(CSetAddr[CSet16+2]);
    musterDec.cMaxPulse = EEPROM.read(CSetAddr[CSet16+3]) + ((high << 8) & 0xFF00);
    if (musterDec.cMaxPulse == 0) {
       musterDec.cMaxPulse = maxPulse;
    }
    musterDec.cMaxPulse = -musterDec.cMaxPulse;
    musterDec.mcMinBitLen = EEPROM.read(CSetAddr[1]);
    if (musterDec.mcMinBitLen == 0) {
        musterDec.mcMinBitLen = mcMinBitLenDef;
    }
}

void initEEPROMconfig(void)
{
	EEPROM.write(addr_features, 0x3F);    	// Init EEPROM with all flags enabled, except MuNoOverflow and toggleBank
	
	for (uint8_t i = 0; i < CSetAnzEE; i++) {
		EEPROM.write(CSetAddr[i], CSetDef[i]);
	}
	EEPROM.write(addr_bank, 0);
	MSG_PRINTLN(F("Init eeprom to defaults"));
}

void initEEPROM(void)
{
  if (EEPROM.read(EE_MAGIC_OFFSET) == VERSION_1 && EEPROM.read(EE_MAGIC_OFFSET+1) == VERSION_2) {
    
  //if (musterDec.MdebEnabled) {
    #ifdef DEBUG
    MSG_PRINTLN(F("Reading values from eeprom"));
    #endif
  //}

  } else {
    initEEPROMconfig();
    //storeFunctions(1, 1, 1);    // Init EEPROM with all flags enabled
    #ifdef CMP_CC1101
       if (EEPROM.read(EE_MAGIC_OFFSET) != VERSION_1) {  // ccFactoryReset nur wenn VERSION_1 nicht passt
          cc1101::ccFactoryReset(true);
       }
    #endif
    EEPROM.write(EE_MAGIC_OFFSET, VERSION_1);
    EEPROM.write(EE_MAGIC_OFFSET+1, VERSION_2);
  }
  callGetFunctions();
}
