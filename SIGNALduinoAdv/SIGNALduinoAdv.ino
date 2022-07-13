/*
*   RF_RECEIVER v4.xx for Arduino
*   Sketch to use an arduino as a receiver/sending device for digital signals
*
*   The Sketch can also encode and send data via a transmitter,
*   while only PT2262 type-signals for Intertechno devices are implemented in the sketch,
*   there is an option to send almost any data over a send raw interface
*   2014-2015  N.Butzek, S.Butzek
*   2016 S.Butzek
*   2020-2022 Ralf9
*
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

* Version from: https://github.com/Ralf9/SIGNALDuino/tree/dev-r422_cc1101

*------------------------------------------------------------------------------------------

*-----  fuer den MapleMini ist bei der Serial und USB Version der core 1.9.0 erforderlich, der core 2.x.x hat noch einen bug in der SerialUSB ------
*/

#include "compile_config.h"
#include <Arduino.h>

#define PROGNAME               " SIGNALduinoAdv "
#define PROGVERS               "4.2.2-dev220712"
#define VERSION_1               0x41
#define VERSION_2               0x2d


	#ifdef MAPLE_SDUINO
		const uint8_t pinSend[] = {10, 17, 13};
		const uint8_t pinReceive[] = {11, 18, 16, 14};
		#define PIN_LED              33
		#define PIN_SEND             17   // gdo0 Pin TX out
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
		#define PIN_WIZ_RST          27
	#elif MAPLE_CUL
		const uint8_t pinSend[] = {10, 17, 13};
		const uint8_t pinReceive[] = {11, 18, 16, 14};
		#define PIN_LED              33
		#define PIN_SEND             17   // gdo0 Pin TX out
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
		#define PIN_WIZ_RST          27
	#elif BLACK_BOARD
		const uint8_t pinSend[] = {33, 3};  // (PB1 PB0)
		const uint8_t pinReceive[] = {11, 20, 0, 1};  // (PA0 PA15 PB11 PB10)
		#define PIN_LED              10 // PA1
		#define PIN_SEND             3  // PB0 gdo0 Pin TX out
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
		#define PIN_WIZ_RST          13  // PC14
	#elif SIGNALESP32
		const uint8_t pinSend[] = {26, 4};
		const uint8_t pinReceive[] = {25, 13, 14, 21};
		#define PIN_LED              2
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
	#elif EVIL_CROW_RF
		const uint8_t pinSend[] = {2, 26};
		const uint8_t pinReceive[] = {4, 25, 14, 21};
		#define PIN_LED              13
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
	#elif ESP32_SDUINO_TEST
		const uint8_t pinSend[] = {26, 22};
		const uint8_t pinReceive[] = {25, 21, 14, 15};
		#define PIN_LED              2
		#define PIN_RECEIVE_A        pinReceive[0]   // gdo2 cc1101 A
		#define PIN_RECEIVE_B        pinReceive[1]   // gdo2 cc1101 B
	#endif

#ifdef MAPLE_Mini
	#define BAUDRATE               115200
	#define FIFO_LENGTH            170
#elif defined(ESP32)
	#define BAUDRATE               115200
	#define FIFO_LENGTH            200      // !! bitte auch das FIFO_LENGTH in der SimpleFIFO_h beachten
#else
	#define BAUDRATE               57600
	#define FIFO_LENGTH            140 // 50
#endif

#define DEBUG                  1

#ifdef MAPLE_WATCHDOG
	#include <IWatchdog.h>
	bool watchRes = false;
#elif WATCHDOG
	#include <avr/wdt.h>
#endif

#ifdef DEBUG_BackupReg
	#include <backup.h>
	uint8_t sichBackupReg = 0;
#endif


//---------------------------------------------------------------------------------------------

#include "cc1101.h"
#include "output.h"
#include "mbus.h"
#include "bitstore4.h"
#include "signalDecoder4.h"
#include "SimpleFIFO.h"

// "Callee" can provide a callback to Caller.
class Callee : public rssiCallbackInterface
{
    public:
    // The callback function that Caller will call.
    uint8_t cbiRssiCallbackFunction()
    {
        return cc1101::getRSSI();
    }
};

//SimpleFIFO<int16_t,FIFO_LENGTH> FiFoA; //store FIFO_LENGTH
//SimpleFIFO<int16_t,FIFO_LENGTH> FiFoB; //store FIFO_LENGTH
SimpleFIFO FiFoA(FIFO_LENGTH);
SimpleFIFO FiFoB(FIFO_LENGTH);
SignalDetectorClass musterDecA;
SignalDetectorClass musterDecB;

Callee rssiCallee;


//---- hardware specifics settings -----------------------------------------
#ifdef MAPLE_Mini
  #include <malloc.h>
  extern char _estack;
  extern char _Min_Stack_Size;
  static char *ramend = &_estack;
  static char *minSP = (char*)(ramend - &_Min_Stack_Size);
  extern "C" char *sbrk(int i);
#elif defined(ESP32)
  void ICACHE_RAM_ATTR sosBlink(void *pArg);
  
  #include <WiFi.h>
  #include <WiFiType.h>
  #include <WiFiManager.h>
  #define WIFI_MANAGER_OVERRIDE_STRINGS
  
  //needed for library
  #include <DNSServer.h>
  
  //const char* ssid = "...";
  //const char* password = "...";
  
  WiFiServer Server(23);  //  port 23 = telnet
  WiFiClient client;
  
  #include "esp_timer.h"
  #include "esp_task_wdt.h"
  
  esp_timer_create_args_t cronTimer_args;
  esp_timer_create_args_t blinksos_args;
  esp_timer_handle_t cronTimer_handle;
  esp_timer_handle_t blinksos_handle;
#else
  #include <TimerOne.h>  // Timer for LED Blinking
#endif

#ifdef LAN_WIZ
  #include <SPI.h>
  #include <Ethernet.h>
  #ifdef MAPLE_CUL
     SPIClass SPI_1(28, 29, 30);
  #endif
  EthernetServer server = EthernetServer(23);
  EthernetClient client;
#endif

#if defined(DEBUG_SERIAL) || defined(SERIAL_USART2)
  #include <HardwareSerial.h>
#endif
#ifdef DEBUG_SERIAL
  HardwareSerial HwSerial(SerialNr);
#endif
#ifdef SERIAL_USART2
  HardwareSerial Serial(USART2);
#endif

#define pulseMin  90
#define maxCmdString 600
#define maxSendPattern 10
#define mcMinBitLenDef   17
//#define ccMaxBuf 64
#define defMaxMsgSize 1500	// selber Wert wie in signalDecoder4.h
#define maxSendEcho 100     // siehe auch in der mbus.h

#define radioOokAsk 1
#define defSelRadio 1	// B
#define defStatRadio 0xFF

//--- EEProm Address
#define EE_MAGIC_OFFSET      0
//#define addr_togglesec       0x3C
#define addr_ccN             0x3D
#define addr_ccmode          0x3E
//#define addr_features2       0x3F
#define addr_bankdescr       0x40    // 0x40-0x47 bis Bank 9 0x88-0x8F  # Bank 0 bis Bank 9, Kurzbeschreibungen (max 8 Zeichen)
//addr statRadio, alt eb - ee, 14.01.21
#define addr_statRadio       0xE0    // A=E0 B=E1 C=E2 D=E3  Bit 0-3 Bank,  1F-Init, Bit 6 = 1 - Fehler bei Erkennung, Bit 6&7 = 1 - Miso Timeout, FF-deaktiviert
#define addr_selRadio        0xE4    // alt EF
#define addr_res_e5          0xE5 // reserve
#define addr_res_e6          0xE6
#define addr_res_e7          0xE7
#define addr_res_e8          0xE8
#define addr_rxRes           0xE9    // bei 0xA5 ist rx nach dem Reset disabled
// CSetAddr[] res ea - fd, alt f0 - fc, 14.01.21
//      addr_features                res mseq led deb red mc mu ms  (7 .. 0)
#define addr_featuresA       0xFE // cc1101 Modul A
#define addr_featuresB       0xFF // cc1101 Modul B
// Ethernet EEProm Address, res c0 - df
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
volatile unsigned long lastTimeA = micros();
volatile unsigned long lastTimeB = micros();
volatile bool lastFifoALowMax = false;
volatile bool lastFifoBLowMax = false;
bool hasCC1101;
bool LEDenabled = true;
//bool toggleBankEnabled = false;
bool RXenabled[] = {false, false, false, false};	// true - enable receive, Zwischenspeicher zum enablereceive merken
volatile bool RXenabledSlowRfA = false;
volatile bool RXenabledSlowRfB = false;
bool unsuppCmd = false;
bool CmdOk = false;
bool FSKdebug = false;
uint8_t MdebFifoLimitA = 120;
uint8_t MdebFifoLimitB = 120;
uint8_t bank = 0;
uint16_t bankOffset = 0;
//uint8_t ccN = 0;
uint8_t ccmode = 0;		// cc1101 Mode: 0 - normal, 1 - FIFO, 2 - FIFO ohne dup, 3 - FIFO LaCrosse, 4 - FIFO LaCrosse, 8 - WMBus, 9 - FIFO mit Debug Ausgaben
uint8_t radionr = defSelRadio;
uint8_t radio_bank[4];
uint8_t ccBuf[4][ccMaxBuf+2];

#if !defined(LAN_WIZ) && !defined(ESP32)
  bool unsuppCmdEnable;
#endif

//--- Ethernet
uint8_t mac[6];
uint8_t ip[4];
uint8_t gateway[4];
uint8_t netmask[4];

//--- FORWARDS @ main -----------------------------------------
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
void cmd_test();
void cmd_Version();
void cmd_writeEEPROM();
void cmd_writePatable();
void changeReceiver();
void enableReceive();
void disableReceive(bool flagCCmode0);
void callGetFunctions();
void getFunctions(bool *ms,bool *mu,bool *mc, bool *red, bool *deb, bool *led, bool *mseq);
void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t deb, int8_t led, int8_t mseq);

void getCSvar();
void getSelRadioBank();
void getRxFifo(uint16_t Boffs);
void print_bank_sum();
void print_Bank();
void en_dis_receiver(bool en_receiver);
void getEthernetConfig(bool flag);
void initEthernetConfig();
void initEEPROM();
void initEEPROMconfig();
void serialEvent();
void HandleCommand();
unsigned long getUptime();
void setCCmode();
void print_radio_sum();
uint16_t getBankOffset(uint8_t tmpBank);
uint8_t radioDetekt(bool confmode, uint8_t Dstat);
void printHex2(const uint8_t hex);
void setHasCC1101(uint8_t val);

//--- platform specific forwards @ main ------------------------------------------
#ifdef LAN_WIZ
  void ethernetLoop();
#endif
#ifdef ESP32
	bool wifiConnected = true;
	inline void WiFiEvent();
	void IRAM_ATTR cronjob(void *pArg);
#else
	void cronjob();
#endif
//--------------------------------------------------------------------------------

//typedef void (* GenericFP)(int); //function pointer prototype to a function which takes an 'int' an returns 'void'
#define cmdAnz 23
const char cmd0[] =  {'?', '?', 'b', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'e', 'e', 'P', 'r', 'R', 'S', 't', 'T', 'V', 'W', 'x', 'X', 'X'};
const char cmd1[] =  {'S', ' ', ' ', 'E', 'D', 'G', 'R', 'S', 'W', ' ', 'C', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'E', 'Q'};
const bool cmdCC[] = {  0,   0,   0,   0,   0,   0,  1,   0,    1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,  0 };
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
		cmd_test, // T
		cmd_Version,	// V
		cmd_writeEEPROM,// W
		cmd_writePatable,// x
		changeReceiver,	// XE
		changeReceiver	// XQ
		};

#define CSetAnz 14	// Anzahl der Konfig Variablen
#define CSetAnzEE 20 // Anzahl der CSetAddr
#define CSet16 13	// Indexpos der 16Bit Konfig Variablen
#define CSccN 0
#define CSccmode 1

//const char *CSetCmd[] = {  ccN        ccmode  mcmbl mscnt fifolimit maxMu..x256 m..Sizex256 fifolimitA maxMu..x256A m..Sizex256A onlyRXB maxnumpat muthreshx256 maxpulse,L, res ,res ,res ,res, res};
const uint8_t CSetAddr[] = {addr_ccN,addr_ccmode, 0xea, 0xeb,    0xec,       0xed,       0xee,      0xef,        0xf0,        0xf1,   0xf2,     0xf3,       0xf4, 0xf5, 0xf6, 0xf7,0xf8,0xf9,0xfa,0xfb};
const uint8_t CSetDef[] =  {   0,          0,        0,    4,     150,          3,          4,       150,           3,           4,      0,        8,          0,    0,    0,  255, 255, 255, 255,255};
// max 17 Zeichen
const char string_0[] PROGMEM = "ccN";
const char string_1[] PROGMEM = "ccmode";
const char string_2[] PROGMEM = "mcmbl";
const char string_3[] PROGMEM = "mscnt";
const char string_4[] PROGMEM = "fifolimit";
const char string_5[] PROGMEM = "maxMuPrintx256";
const char string_6[] PROGMEM = "maxMsgSizex256";
const char string_7[] PROGMEM = "fifolimitA";
const char string_8[] PROGMEM = "maxMuPrintx256A";
const char string_9[] PROGMEM = "maxMsgSizex256A";
const char string_10[] PROGMEM = "onlyRXB";
const char string_11[] PROGMEM = "maxnumpat";
const char string_12[] PROGMEM = "muthreshx256";
const char string_13[] PROGMEM = "maxpulse";

const char * const CSetCmd[] PROGMEM = { string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11, string_12, string_13};

bool command_available=false;

#ifdef ESP32
const char sos_sequence[] = "0101010001110001110001110001010100000000";
const char boot_sequence[] = "00010100111";

void ICACHE_RAM_ATTR sosBlink (void *pArg) {
  static uint8_t pos = 0;
  const char* pChar;
  pChar = (const char*)pArg;      //OK in both C and C++

  digitalWrite(PIN_LED, pChar[pos] == '1' ? HIGH : LOW);
  pos++;
  if (pos == sizeof(pChar) * sizeof(pChar[1]))
    pos = 0;
}

//--------------------------------------------------------------------------------
//--- gets called, when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) 
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  //ticker.attach(0.2, tick);
}

bool shouldSaveConfig = false;

void saveConfigCallback() {
	Serial.println("Should save config");
	shouldSaveConfig = true;
}

bool saveIp(IPAddress IP, uint8_t EE_pos) {
	bool flag = false;
	uint8_t ii;
	for (uint8_t i = 0; i < 4; i++) {
		ii = IP[i];
		if (ii != tools::EEread(EE_pos+i)) {
			//Serial.print("ne ");
			tools::EEwrite(EE_pos+i,ii);
			flag = true;
		}
		//Serial.println(ii);
	}
	return flag;
}

WiFiManager wifiManager;

#endif

void setup() {
#ifdef MAPLE_Mini
	pinAsOutput(PIN_WIZ_RST);
#ifndef LAN_WIZ
	digitalWrite(PIN_WIZ_RST, HIGH);
#endif
#endif
#ifdef DEBUG_BackupReg
	sichBackupReg = (uint8_t)getBackupRegister(RTC_BKP_INDEX);
#endif
	tools::EEbufferFill();
	getEthernetConfig(true);
	pinAsOutput(PIN_LED);
#ifdef LAN_WIZ
	digitalWrite(PIN_LED, LOW);
	digitalWrite(PIN_WIZ_RST, LOW);		// RESET should be heldlowat least 500 us for W5500
	delayMicroseconds(500);
	digitalWrite(PIN_WIZ_RST, HIGH);
  #ifdef MAPLE_CUL
	Ethernet.init(SPI_1, 31);
  #endif
	if (ip[3] != 0) {
		Ethernet.begin(mac, ip, gateway, netmask);
	}
	else {  // DHCP
		Ethernet.begin(mac);
	}
	server.begin();		// start listening for clients

#elif defined(ESP32)
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Serial.println(F("-----WiFi connected------"));
		Server.begin();  // start telnet server
		Server.setNoDelay(true);
  #if ESP_IDF_VERSION_MAJOR < 4
	}, WiFiEvent_t::SYSTEM_EVENT_STA_CONNECTED);
  #else
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  #endif	
	WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
		Server.stop();  // end telnet server
		Serial.print(F("WiFi lost connection. Reason: "));
	  #if ESP_IDF_VERSION_MAJOR < 4
		Serial.println(info.disconnected.reason);
		if (info.disconnected.reason == 4) {
	  #else
        Serial.println(info.wifi_sta_disconnected.reason);
		if (info.wifi_sta_disconnected.reason == 4) {
	  #endif
			Serial.println(F("-----WiFi try reconnect------"));
			WiFi.disconnect();
			WiFi.begin();
		}
		wifiConnected = false;
  #if ESP_IDF_VERSION_MAJOR < 4
	}, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
  #else
    }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  #endif
	
	blinksos_args.callback = sosBlink;
	blinksos_args.dispatch_method = ESP_TIMER_TASK;
	blinksos_args.name = "blinkSOS";
	blinksos_args.arg = (void *)boot_sequence;
	esp_timer_create(&blinksos_args, &blinksos_handle);
	esp_timer_start_periodic(blinksos_handle, 300000);
	
	WiFi.mode(WIFI_STA);
	
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	while (!Serial)
		delay(90);

	Serial.println(F("\nstarted"));
	
	//WiFiManager wifiManager;
	
	//set config save notify callback
	wifiManager.setSaveConfigCallback(saveConfigCallback);
	
	int16_t serTimeout = 250;
	bool serCmdAvailable = false;
	char IBuf[6];
	IBuf[0] = 0;
	uint8_t Iidx = 0;
	Serial.print(F("\nserial menue? (enter 'cmd') ")); Serial.print(F("Timeout:")); Serial.println(serTimeout * 10);
	while (serTimeout > 0) {
		if ( Serial.available() && Iidx < 5)
		{
			IBuf[Iidx] = (char)Serial.read();
			switch (IBuf[Iidx])
			{
				case '\n':
				case '\r':
				case '\0':
					IBuf[Iidx] = 0;
					if (strcmp(IBuf,"cmd") == 0) {
						serCmdAvailable = true;
					}
					serTimeout = 1;
			}
			Iidx++;
		}
		delay(10);
		serTimeout--;
	}
	
	if (serCmdAvailable == true){
		Serial.println(F("serial command menue"));
		Serial.println(F("i - printDiag"));
		Serial.print(F("c - change DHCP <> StaticIP, act: "));
		if (ip[3] == 0) {
			Serial.println(F("DHCP"));
		}
		else {
			Serial.println(F("static"));
		}
		Serial.println(F("R - resetWifiSettings"));
		Serial.println(F("q - quit"));
		
		Iidx = 0;
		bool serExitFlag = false;
		while (serExitFlag == false) {
			if ( Serial.available() && Iidx < 5) {
				IBuf[Iidx] = (char)Serial.read();
				switch (IBuf[Iidx])
				{
					case '\n':
					case '\r':
					case '\0':
						//Serial.print(Iidx);
						//Serial.print(" buf=");
						//Serial.println(IBuf);
						if (IBuf[0] == 'q') {
							serExitFlag = true;
						}
						else if (IBuf[0] == 'i') {
							WiFi.printDiag(Serial);
							Serial.println("");
						}
						else if (IBuf[0] == 'c') {
							if (ip[3] == 0) {      // DHCP
								ip[3] = ip_def[3];
								Serial.println(F("new: static"));
							}
							else {
								ip[3] = 0;
								Serial.println(F("new: DHCP"));
							}
						}
						else if (IBuf[0] == 'R') {
							Serial.println(F("Return to AP-mode, because reset command received."));
							//--- reset saved settings
							wifiManager.resetSettings();
							serExitFlag = true;
						}
						Iidx = 0;
						break;
					default:
						Iidx++;
				}
			}
		}
	}
	else {
		Serial.print(F("serial command, timeout "));
		Serial.print(Iidx);
		Serial.print(" buf=");
		Serial.println(IBuf);
	}
	
	if (ip[3] != 0) { // kein DHCP
		wifiManager.setSTAStaticIPConfig(ip,gateway,netmask);
		//wifiManager.setShowStaticFields(true);
	}
	
		//--- set callback that gets called, when connecting to previous WiFi fails, and enters AP-mode
		wifiManager.setAPCallback(configModeCallback);

		if ( !wifiManager.autoConnect("ESP32DuinoConfig",NULL) ) 
		{
			Serial.println(F("failed to connect and hit timeout"));
			//--- reset and try again, or maybe put it to deep sleep
			ESP.restart();
			delay(3000);
		}
    
    	//--- if you get here you have connected to the WiFi
    	Serial.println(F("WifiManager had connected ...yeey :)"));
	
	//WiFi.config(IPAddress(192,168,0,47), IPAddress(192,168,0,191), IPAddress(255,255,255,0));
	//WiFi.begin(ssid, password);
	//uint8_t i = 0;
	//while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
	//Serial.print("i="); Serial.println(i);
	Serial.println(F("\nlocal ip"));
	Serial.println(WiFi.localIP());
	Serial.println(WiFi.gatewayIP());
	Serial.println(WiFi.subnetMask());
	
	if (shouldSaveConfig) {
		Serial.println(F("\nsave ip"));
		bool saveFlag;
		saveFlag = saveIp(WiFi.localIP(), EE_IP4_ADDR) + saveIp(WiFi.gatewayIP(), EE_IP4_GATEWAY) + saveIp(WiFi.subnetMask(), EE_IP4_NETMASK);
		if (saveFlag) {
			tools::EEstore();
			Serial.println(F("IPaddress changed -> save"));
			getEthernetConfig(false);
		}
		
	}
	
	Server.begin();    // start listening for clients
	
#else  // MapleMini USB
	if (tools::EEread(addr_rxRes) == 0xA5) {	// wenn A5 dann bleibt rx=0 und es gibt keine "Unsupported command" Meldungen
		unsuppCmdEnable = false;
	}
	else {
		unsuppCmdEnable = true;
	}
	digitalWrite(PIN_LED, HIGH);
  #ifdef DEBUG_SERIAL
	HwSerial.begin(BAUDRATE);
  #endif
	Serial.begin(BAUDRATE);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB
	}
	/*for (uint8_t sw=0;sw<255;sw++ ) {
		delay(10);
		if (Serial) {
			break;
		}
	}*/
	digitalWrite(PIN_LED, LOW);
  #ifdef DEBUG_SERIAL
	HwSerial.println(F("serial init ok"));
  #endif
#endif

#ifdef DEBUG_BackupReg
	MSG_PRINT(F("BackupReg = "));
	MSG_PRINTLN(sichBackupReg);
	setBackupReg(0);
#endif
	if (musterDecB.MdebEnabled) {
		DBG_PRINTLN(F("Using sFIFO"));
	}
#ifdef MAPLE_WATCHDOG
	if (IWatchdog.isReset(true)) {
		MSG_PRINTLN(F("Watchdog caused a reset"));
		watchRes = true;
	}
	else {
		watchRes = false;
	}
	IWatchdog.begin(20000000);	// Init the watchdog timer with 20 seconds timeout
	if (IWatchdog.isEnabled()) {
		MSG_PRINTLN(F("Watchdog enabled"));
	}
#elif WATCHDOG
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
	pinAsInput(PIN_RECEIVE_A);
	pinAsInput(PIN_RECEIVE_B);
	pinAsInput(pinReceive[2]);
	pinAsInput(pinReceive[3]);
	pinAsInput(pinSend[0]);    // gdo0Pin, sicherheitshalber bis zum CC1101 init erstmal input
	pinAsInput(pinSend[1]);
	pinAsInput(pinSend[2]);
	
	// CC1101
#ifdef WATCHDOG
	wdt_reset();
#endif
	setHasCC1101(tools::EEread(CSetAddr[10]));	// onlyRXB - keine cc1101
#ifdef CMP_CC1101
	if (hasCC1101) cc1101::setup();
#endif
  	initEEPROM();
	uint8_t statRadio;
#ifdef CMP_CC1101
  if (hasCC1101) {
	MSG_PRINTLN(F("CCInit"));
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
  }
  else {
	radio_bank[0] = defStatRadio;
	radio_bank[1] = 0;
	radio_bank[2] = defStatRadio;
	radio_bank[3] = defStatRadio;
  }
	
	// Connect the rssiCallback
	musterDecA.rssiConnectCallback(&rssiCallee);
	musterDecB.rssiConnectCallback(&rssiCallee);
#endif 

	if (musterDecB.MdebEnabled) {
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
#elif defined(ESP32)
	MSG_PRINTER.setTimeout(400);
	
	cronTimer_args.callback = cronjob;
	cronTimer_args.name = "cronTimer";
	cronTimer_args.dispatch_method = ESP_TIMER_TASK;
	esp_timer_create(&cronTimer_args, &cronTimer_handle);
	
	esp_timer_start_periodic(cronTimer_handle, 31000);
	esp_timer_stop(blinksos_handle);
#else
	Timer1.initialize(31*1000); //Interrupt wird jede 31 Millisekunden ausgeloest
	Timer1.attachInterrupt(cronjob);
#endif
	cmdstring.reserve(maxCmdString);

  if (hasCC1101) {
	for (radionr = 0; radionr < 4; radionr++) {	// enableReceive bei allen korrekt erkannten radios denen eine Bank zugeordnet ist
		statRadio = radio_bank[radionr];
		if (statRadio < 10) {
			bankOffset = getBankOffset(statRadio);
			ccmode = tools::EEbankRead(addr_ccmode);
			if (ccmode == 8) {
				mbus_init(tools::EEbankRead(addr_ccN));
			}
			if (radionr > 1 || ccmode > 0 || cc1101::regCheck()) {
				if (tools::EEread(addr_rxRes) != 0xA5) {	// wenn A5 dann bleibt rx=0
					en_dis_receiver(true);
				}
				if (radionr <= 1 && ccmode == 0) {
					pinAsOutput(pinSend[radionr]);
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
  }
  else {	// onlyRXB
	radionr = 1;
	ccmode = tools::EEread(addr_ccmode);
	if (tools::EEread(addr_rxRes) != 0xA5) {	// wenn A5 dann bleibt rx=0
		en_dis_receiver(true);
	}
	if (ccmode == 0) {
		pinAsOutput(pinSend[radionr]);
	}
  }
	MSG_PRINTLN("");
	getSelRadioBank();
} //---  of setup


//--------------------------------------------------------------------------------
#ifdef ESP32
void IRAM_ATTR cronjob(void *pArg) {
	cli();
#else // MapleMini
void cronjob() {
	noInterrupts();
#endif
	static uint16_t cnt0 = 0;
	static uint8_t cnt1 = 0;
	unsigned long durationA;
	unsigned long durationB;
	
	if (RXenabledSlowRfA) {
	  durationA = micros() - lastTimeA;
	  if (durationA > maxPulse) { //Auf Maximalwert pruefen.
		 int16_t sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE_A)) { // Wenn jetzt low ist, ist auch weiterhin low
			if (lastFifoALowMax == false) { // der letzte FiFo war nicht maxPulse
				FiFoA.enqueue(-sDuration);
				lastFifoALowMax = true;
			}
		 }
		 else {
			FiFoA.enqueue(sDuration);
		 }
		 lastTimeA = micros();
	  }
	}
	
	if (RXenabledSlowRfB) {
	  durationB = micros() - lastTimeB;
	  if (durationB > maxPulse) { //Auf Maximalwert pruefen.
		 int16_t sDuration = maxPulse;
		 if (isLow(PIN_RECEIVE_B)) { // Wenn jetzt low ist, ist auch weiterhin low
			if (lastFifoBLowMax == false) { // der letzte FiFo war nicht maxPulse
				FiFoB.enqueue(-sDuration);
				lastFifoBLowMax = true;
			}
		 }
		 else {
			FiFoB.enqueue(sDuration);
		 }
		 lastTimeB = micros();
	  }
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

//--------------------------------------------------------------------------------
void setHasCC1101(uint8_t val) {
	if (val == 1) {
		hasCC1101 = false;	// onlyRXB - keine cc1101
	}
	else {
		hasCC1101 = true;
	}
	musterDecA.hasCC1101 = hasCC1101;
	musterDecB.hasCC1101 = hasCC1101;
}

#ifdef DEBUG_BackupReg
void setBackupReg(uint32_t n) {
	enableBackupDomain();
	setBackupRegister(RTC_BKP_INDEX, n);
	disableBackupDomain();
}
#endif

//--------------------------------------------------------------------------------
void loop() {
	static int16_t aktVal=0;
	bool state;
	uint8_t fifoCount;
	
#ifdef SERIAL_USART2
	serialEvent();
#endif
#ifdef LAN_WIZ
	serialEvent();
	ethernetLoop();
#endif
#ifdef ESP32
	if (wifiConnected == false) {
		while (WiFi.status() != WL_CONNECTED) {
			yield();
		}
		wifiConnected = true;
		Serial.println("*******Connected*****");
		Server.begin();    // start listening for clients
	}
	serialEvent();
	WiFiEvent();
#endif
	if (command_available == true) {
		command_available=false;
		if (isAlpha(cmdstring.charAt(0)) || cmdstring.charAt(0) == '?') {
			HandleCommand();
			if (LEDenabled) {
				blinkLED=true;
			}
		}
		if (!command_available) { cmdstring = ""; }
	}
#ifdef MAPLE_WATCHDOG
	IWatchdog.reload();
#elif WATCHDOG
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
	if (radionr == 0) {
		musterDecA.printMsgSuccess = false;
		while (FiFoA.count() > 0) { // Puffer auslesen und an Dekoder uebergeben
			aktVal=FiFoA.dequeue();
			//MSG_PRINTLN(aktVal, DEC);
			state = musterDecA.decode(&aktVal);
			if (musterDecA.MdebEnabled && musterDecA.printMsgSuccess) {
				fifoCount = FiFoA.count();
				if (fifoCount > MdebFifoLimitA) {
					MSG_PRINT(F("MFa="));
					MSG_PRINTLN(fifoCount, DEC);
				}
			}
			if (musterDecA.printMsgSuccess && LEDenabled) {
				blinkLED=true; //LED blinken, wenn Meldung dekodiert
			}
			musterDecA.printMsgSuccess = false;
		}
	} else if (radionr == 1) {
		musterDecB.printMsgSuccess = false;
		while (FiFoB.count() > 0) { //Puffer auslesen und an Dekoder uebergeben
			aktVal=FiFoB.dequeue();
			//MSG_PRINTLN(aktVal, DEC);
			state = musterDecB.decode(&aktVal);
			if (musterDecB.MdebEnabled && musterDecB.printMsgSuccess) {
				fifoCount = FiFoB.count();
				if (fifoCount > MdebFifoLimitB) {
					MSG_PRINT(F("MF="));
					MSG_PRINTLN(fifoCount, DEC);
				}
			}
			if (musterDecB.printMsgSuccess && LEDenabled) {
				blinkLED=true; //LED blinken, wenn Meldung dekodiert
			}
			musterDecB.printMsgSuccess = false;
		}
	}
  }
  else if (ccmode == 8) {
	if (RXenabled[radionr] == true) {
		mbus_task(bankoff + addr_ccN);
	}
  }
  else if (ccmode < 15) {
	if (RXenabled[radionr] == true) {
		getRxFifo(bankoff);
	}
  }
 }
 radionr = remRadionr;
 ccmode = remccmode;
 yield();
}

//--------------------------------------------------------------------------------
void getRxFifo(uint16_t Boffs) {
	uint8_t fifoBytes;
	bool dup;		// true bei identischen Wiederholungen bei readRXFIFO

	if (isHigh(pinReceive[radionr])) {  // wait for CC1100_FIFOTHR given bytes to arrive in FIFO
#ifdef DEBUG_BackupReg
	setBackupReg(1);
#endif
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
			if ((tools::EEread(Boffs + 2 +CC1101_PKTCTRL1) & 4) == 4) {
				appendRSSI = true;
			}
			else {
				RSSI = cc1101::getRSSI();
			}
			
			if (ccmode == 9) {
				if (fifoBytes > 1 && fifoBytes < 0x80) {
					fifoBytes--;
				}
				MSG_PRINT(F("RX("));
				MSG_PRINT(fifoBytes);
				MSG_PRINT(F(") "));
			}
			if (fifoBytes < 0x80) {	// RXoverflow?
				if (fifoBytes > ccMaxBuf) {
					fifoBytes = ccMaxBuf;
				}
				dup = cc1101::readRXFIFOdup(fifoBytes, ccmode, appendRSSI);
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
#ifdef DEBUG_BackupReg
	setBackupReg(2);
#endif
	}
}

//========================= Pulseauswertung ================================================

#ifdef ESP32
void IRAM_ATTR handleInterruptA() {
  cli();
#else // MapleMini
void handleInterruptA() {
  noInterrupts();
#endif
  const unsigned long Time=micros();
  const unsigned long  duration = Time - lastTimeA;
  lastTimeA = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
	int16_t sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int16_t(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (isHigh(PIN_RECEIVE_A)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration=-sDuration;
    }
    FiFoA.enqueue(sDuration);
    lastFifoALowMax = false;
  } // else => trash

#ifdef MAPLE_Mini
  interrupts();
#else
  sei();
#endif
}

#ifdef ESP32
void IRAM_ATTR handleInterruptB() {
  cli();
#else // MapleMini
void handleInterruptB() {
  noInterrupts();
#endif
  const unsigned long Time=micros();
  const unsigned long  duration = Time - lastTimeB;
  lastTimeB = Time;
  if (duration >= pulseMin) {//kleinste zulaessige Pulslaenge
	int16_t sDuration;
    if (duration < maxPulse) {//groesste zulaessige Pulslaenge, max = 32000
      sDuration = int16_t(duration); //das wirft bereits hier unnoetige Nullen raus und vergroessert den Wertebereich
    }else {
      sDuration = maxPulse; // Maximalwert set to maxPulse defined in lib.
    }
    if (isHigh(PIN_RECEIVE_B)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration=-sDuration;
    }
    FiFoB.enqueue(sDuration);
    lastFifoBLowMax = false;
  } // else => trash

#ifdef MAPLE_Mini
  interrupts();
#else
  sei();
#endif
}

//--------------------------------------------------------------------------------
void enableReceive() {
  if (RXenabled[radionr] == true) {
   if (ccmode == 0 && radionr == 0) {
     attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE_A), handleInterruptA, CHANGE);
     RXenabledSlowRfA = true;
   }
   if (ccmode == 0 && radionr == 1) {
     attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE_B), handleInterruptB, CHANGE);
     RXenabledSlowRfB = true;
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

//--------------------------------------------------------------------------------
void disableReceive(bool flagCCmode0) {
  if (ccmode == 0 || flagCCmode0 == true) {
    if (radionr == 0) {
      detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE_A));
      RXenabledSlowRfA = false;
      FiFoA.flush();
    }
    else if (radionr == 1) {
      detachInterrupt(digitalPinToInterrupt(PIN_RECEIVE_B));
      RXenabledSlowRfB = false;
      FiFoB.flush();
    }
  }
  #ifdef CMP_CC1101
  if (hasCC1101) cc1101::ccStrobe_SIDLE();	// Idle mode
  #endif
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
	uint8_t pin_send = pinSend[radionr];

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
		isLow ? digitalLow(pin_send): digitalHigh(pin_send);
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
		musterDecB.decode(&dur);
	}
  }
	//MSG_PRINTLN("");
}

//--------------------------------------------------------------------------------
//SM;R=2;C=400;D=AFAFAF;
void send_mc(const uint8_t startpos,const uint8_t endpos, const int16_t clock)
{
	int8_t b;
	char c;
	//digitalHigh(PIN_SEND);
	//delay(1);
	uint8_t bit;
	uint8_t pin_send = pinSend[radionr];

	unsigned long stoptime =micros();
	for (uint8_t i = startpos; i <= endpos; i++) {
		c = cmdstring.charAt(i);
		b = ((byte)c) - (c <= '9' ? 0x30 : 0x37);

		for (bit = 0x8; bit>0; bit >>= 1) {
			for (byte i = 0; i <= 1; i++) {
				if ((i == 0 ? (b & bit) : !(b & bit)))
					digitalLow(pin_send);
				else
					digitalHigh(pin_send);
				
					stoptime += clock;
					while (stoptime > micros())
						;
			}
			
		}
		
	}
	// MSG_PRINTLN("");
}

//--------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------
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
	bool sendRadio = 1;
	
	if (cmdstring.length() < 20) {
		MSG_PRINTLN(F("send cmd is to short!"));
		return;
	}
	if (cmdstring.charAt(2) == 'A') {
#ifdef DEBUGSENDCMD
		MSG_PRINTLN(F("send radioA"));
#endif
		sendRadio = 0;
	}
	tmpBank = radio_bank[sendRadio];
	if (tmpBank > 9) {
		MSG_PRINT(F("Radio "));
		MSG_WRITE(sendRadio + 'A');
		MSG_PRINTLN(F(" is not active!"));
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
	radionr = sendRadio;

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

	disableReceive(false);

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
			else {
				startdata = -1;
				break;
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
				digitalLow(pinSend[radionr]);
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
		musterDecB.decode(&dur);
	  }
	}
	
//#ifndef SENDTODECODER
	enableReceive();	// enable the receiver
	MSG_PRINTLN("");
//#endif
	ccmode = remccmode;
	radionr = remRadionr;
}

//--------------------------------------------------------------------------------
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
		
		if (cmdstring.charAt(startdata) == 'b') {
			mbus_send(startdata);
			return;
		}
		
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
			disableReceive(false);
			for (uint8_t i = 0; i < repeats; i++) {
				if (cc1101::setTransmitMode() == false) {
					startdata = -1;
					break;
				}
				cc1101::sendFIFO(startdata, enddata);
			}
			MSG_PRINT(cmdstring); // echo
			MSG_PRINT(F("Marcs="));
			uint8_t marcstate = cc1101::getMARCSTATE();
			MSG_PRINTLN(marcstate);  // 13 - rx
			if (marcstate != 13 && RXenabled[radionr] == true) {
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

void HandleCommand()
{
//	uint8_t reg;
//	uint8_t val;
	uint8_t i;
	
#ifdef DEBUG_BackupReg
	setBackupReg(8);
#endif
	for (i=0; i < cmdAnz; i++) {
	  if (hasCC1101 || cmdCC[i] == 0) {
		if (cmdstring.charAt(0) == cmd0[i]) {
			if (cmd1[i] == ' ' || (cmdstring.charAt(1) == cmd1[i])) {
				break;
			}
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
#if !defined(LAN_WIZ) && !defined(ESP32)
	if (unsuppCmd && unsuppCmdEnable) {
#else
	if (unsuppCmd) {
#endif
		MSG_PRINTLN(F("Unsupported command"));
#ifdef DEBUG_SERIAL
		HwSerial.println(F("wr: Unsupported command"));
#endif
	}
	else if (CmdOk) {
		MSG_PRINTLN(F("ok"));
#ifdef DEBUG_SERIAL
		HwSerial.println(F("wr: ok"));
#endif
	}
#ifdef DEBUG_BackupReg
	setBackupReg(9);
#endif
}

//--------------------------------------------------------------------------------
void cmd_help_S()	// get help configvariables
{
	char buffer[18];
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

//--------------------------------------------------------------------------------
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
		else if (cmdstring.charAt(2) == '-' && bank > 0 && bank != remBank) {  // Bank deaktivieren (ungueltig)
			for (uint8_t i = 0; i < 4; i++) {
				if ((radio_bank[i] & 0x0F) == bank) {  // testen ob Bank nicht aktiv ist
					posDigit = 255;
					break;
				}
			}
			if (posDigit != 255) {
				uint16_t tmpBankOffset = getBankOffset(bank);
				tools::EEwrite(tmpBankOffset, 255);
				tools::EEwrite(tmpBankOffset+1, 255);
				tools::EEstore();
				MSG_PRINT(F("Bank "));
				MSG_PRINT(bank);
				MSG_PRINTLN(F(" clear"));
			}
			bank = remBank;
			return;
		}
		uint8_t bankOld_radio = radio_bank[radionr];
		radio_bank[radionr] = bank;
		uint16_t bankOffsetOld = bankOffset;
		bankOffset = getBankOffset(bank);
		if (bank == 0 || cmdstring.charAt(0) == 'e' || (tools::EEbankRead(0) == bank && tools::EEbankRead(1) == (255 - bank))) {
		  if (cmdstring.charAt(posDigit+1) == 'f') {
			uint8_t ccmodeOld = ccmode;
			uint16_t bankOffsetOld_radio = getBankOffset(bankOld_radio);
			uint8_t ccmodeOld_radio = tools::EEread(bankOffsetOld_radio + addr_ccmode);
			ccmode = tools::EEbankRead(addr_ccmode);
			if (ccmodeOld_radio > 0 && ccmodeOld_radio < 5 && ccmode > 0 && ccmode < 5) {
				cc1101::ccStrobe_SIDLE();	// Idle mode
				uint8_t val;
				uint8_t n = 0;
				for (uint8_t i = 0; i <= 0x28; i++) {
					val = tools::EEbankRead(2 + i);
					if (val != tools::EEread(bankOffsetOld_radio + 2 + i)) {
						n++;
						cc1101::writeReg(i,val);
					}
				}
				cc1101::flushrx();
				cc1101::setReceiveMode();
				MSG_PRINT("fn=");
				MSG_PRINT(n);
				MSG_PRINT(" ");
				print_Bank();
			}
			else {
				bankOffset = bankOffsetOld;
				bank = remBank;
   				radio_bank[radionr] = bankOld_radio;
				if (remRadionr != 255) {
					radionr = remRadionr;
				}
				ccmode = ccmodeOld;
				MSG_PRINTLN(F("ccmode not valid (must 1-4)"));
			}
		  }
		  else {
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
				if (hasCC1101) cc1101::CCinit();
				if (ccmode == 8) {
					mbus_init(tools::EEbankRead(addr_ccN));
				}
				setCCmode();
			}
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

//--------------------------------------------------------------------------------
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
void print_ccconf(uint16_t bankOffs)
{
	char hexString[6];
	
	if (hasCC1101 == false) {
		return;
	}
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

//--------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------
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
			  for (j = 0; j < 8; j++) {
					ch = tools::EEread(addr_bankdescr + (i * 8) + j);
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
			  if (ccmodeStr[i2] == '0' && j == 0) { // ccmode = 0 und keine gueltige Kurzbeschreibung
				strcpy(bankStr, "SlowRF");
				j = 6;
			  }
			if (j > 0) {
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

//--------------------------------------------------------------------------------
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
		if (i == radionr) {
			MSG_PRINT(F("*"));
		}
	}
	MSG_PRINTLN("");
}

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
void print_mac(uint8_t mac[])
{
	for (uint8_t i = 0; i < 6; i++) {
		printHex2(mac[i]);
		if (i < 5) {
			MSG_PRINT(F(":"));
		}
	}
}

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
void print_ip(uint8_t ip[])
{
	for (uint8_t i = 0; i < 4; i++) {
		MSG_PRINT(ip[i]);
		if (i < 3) {
			MSG_PRINT(F("."));
		}
	}
}

//--------------------------------------------------------------------------------
void cmd_Version()	// V: Version
{
	MSG_PRINT(F("V " PROGVERS  PROGNAME));

#ifdef ESP32
	MSG_PRINT(F("ESP32 "));
#elif LAN_WIZ
	MSG_PRINT(F("LAN "));
#endif

	if (hasCC1101) {
	    MSG_PRINT(F("cc1101 "));
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
#ifdef MAPLE_WATCHDOG
	if (watchRes) {
		MSG_PRINT(F("wr "));
	}
#endif
	if (tools::EEread(addr_rxRes) == 0xA5) {
		MSG_PRINT(F("irx0 "));
	}
#ifdef DEBUG_BackupReg
	MSG_PRINT(F("-"));
	MSG_PRINT(sichBackupReg,HEX);
#endif
	MSG_PRINTLN(F("- compiled at " __DATE__ " " __TIME__));
}

//--------------------------------------------------------------------------------
void cmd_freeRam()	// R: FreeMemory
{
#ifdef MAPLE_Mini
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();
	MSG_PRINTLN(((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks);
#elif defined(ESP32)
	MSG_PRINTLN(ESP.getFreeHeap());
#else
	extern int __heap_start, *__brkval;
	int v;
	MSG_PRINTLN((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
#endif
}

//--------------------------------------------------------------------------------
void cmd_send()
{
	/*if (musterDecB.getState() != searching )
	{
		command_available=true;
	} else {*/
		if (cmdstring.charAt(1) != 'N') {
			send_cmd(); // Part of Send
		}
		else {
			if (hasCC1101) {
				send_ccFIFO();
			}
			else {
				unsuppCmd = true;
			}
		}
	//}
}

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
void cmd_uptime()	// t: Uptime
{
	MSG_PRINTLN(getUptime());
}

//--------------------------------------------------------------------------------
void cmd_test()
{
	if (cmdstring.charAt(1) == 'd') {
		FSKdebug = false;
	}
	else if (cmdstring.charAt(1) == 'D') {
		FSKdebug = true;
	}
	else {
	#ifdef ARDUINO
		MSG_PRINT(F("a="));
		MSG_PRINT(ARDUINO);
		MSG_PRINT(F(" "));
	#endif
	#ifdef STM32_CORE_VERSION_MAJOR
		MSG_PRINT(F("STM32coreVer="));
		MSG_PRINT(STM32_CORE_VERSION_MAJOR);
		MSG_PRINT(F("."));
		MSG_PRINT(STM32_CORE_VERSION_MINOR);
		MSG_PRINT(F("."));
		MSG_PRINT(STM32_CORE_VERSION_PATCH);
		MSG_PRINT(F(" "));
	#elif ESP_IDF_VERSION_MAJOR
		MSG_PRINT(F("ESP_IDF_VER="));
		MSG_PRINT(ESP_IDF_VERSION_MAJOR);
		MSG_PRINT(F("."));
		MSG_PRINT(ESP_IDF_VERSION_MINOR);
		MSG_PRINT(F("."));
		MSG_PRINT(ESP_IDF_VERSION_PATCH);
		MSG_PRINT(F(" "));
	#endif
	}
	MSG_PRINT(F("FSKdebug="));
	MSG_PRINTLN(FSKdebug);
}

//--------------------------------------------------------------------------------
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

	uint8_t CWccreset = tools::EEbankRead(addr_CWccreset);
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
		reg = tools::cmdstringPos2int(pos);
		val = tools::cmdstringPos2int(pos+2);
		if (reg > 0x47) {
			break;
		}
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
			if (ccmode == 8) {
				mbus_init(tmp_ccN);
			}
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
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
           if (hasCC1101 && reg > 1 && reg <= 0x2A) {  // nur bis cc1101_reg 0x28
              cc1101::writeReg(reg-2, val);
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

//-- todo evtl in eigene lib, aber wie ist dann in der lib der Zugriff auf die arrays mac, ip, gateway und netmask?
//--------------------------------------------------------------------------------
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
		if (ip[3] == 0) {
			MSG_PRINT(F(" DHCP"));
		}
		else {
			MSG_PRINT(F("  gw = "));
			print_ip(gateway);
			MSG_PRINT(F("  nm = "));
			print_ip(netmask);
		}
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------
void cmd_configFactoryReset()	// eC - initEEPROMconfig
{
	uint8_t statRadio;
	
	initEEPROMconfig();
	callGetFunctions();
	radionr = defSelRadio;
	if (hasCC1101 == false) {	// onlyRXB
		tools::EEwrite(CSetAddr[10], 1);
		tools::EEwrite(addr_bankdescr, 0);
		tools::EEstore();
		statRadio = 0;
	}
	else {
		statRadio = tools::EEread(addr_statRadio + defSelRadio);
		statRadio = radioDetekt(false, statRadio);
	}
	if (statRadio == 0) {				// ist dem Radio Bank 0 zugeordnet?
		getSelRadioBank();
		if (hasCC1101) cc1101::CCinit_reg();
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

//--------------------------------------------------------------------------------
void cmd_ccFactoryReset()	// e<0-9>
{
         if (cmdstring.charAt(0) == 'e' && isDigit(cmdstring.charAt(1))) {
            cmd_bank();
         }
         if (hasCC1101) {
            cc1101::ccFactoryReset(true);
            cc1101::CCinit();
         }
         tools::EEbankWrite(addr_ccN, 0);
         //ccN = 0;
         ccmode = 0;
         tools::EEbankWrite(addr_ccmode, ccmode);
         setCCmode();
         tools::EEwrite(addr_bankdescr + (bank * 8), 0);
         if (bank > 0) {
            tools::EEbankWrite(0, bank);
            tools::EEbankWrite(1, (255 - bank));
         }
         tools::EEstore();
         if (cmdstring.charAt(0) == 'e') {
            print_Bank();
         }
}

//--------------------------------------------------------------------------------
inline void getConfig()
{
  SignalDetectorClass *SDptr;
  
  if (radionr == 0) {
    SDptr = &musterDecA;
  }
  else {
    SDptr = &musterDecB;
  }
  
  if (ccmode == 0 || ccmode == 15) {
   if (radionr == 0) {
      MSG_PRINT(F("A: "));
   }
   if (SDptr->MSeqEnabled == 0 || SDptr->MSenabled == 0) {
      MSG_PRINT(F("MS="));
      MSG_PRINT(SDptr->MSenabled,DEC);
   }
   else if (SDptr->MSenabled) {
      MSG_PRINT(F("MSEQ=1"));
   }
   MSG_PRINT(F(";MU="));
   MSG_PRINT(SDptr->MUenabled, DEC);
   MSG_PRINT(F(";MC="));
   MSG_PRINT(SDptr->MCenabled, DEC);
   MSG_PRINT(F(";Mred="));
   MSG_PRINT(SDptr->MredEnabled, DEC);
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
   if (RXenabled[radionr] == false) {
      MSG_PRINT(F(";RX=0"));
   }
//   if (toggleBankEnabled == true) {
//      MSG_PRINT(F(";toggleBank=1"));
//   }
  if (ccmode == 0 || ccmode == 15) {
   MSG_PRINT(F("_MScnt="));
   MSG_PRINT(SDptr->MsMoveCountmax, DEC);
   if (SDptr->maxMuPrint < SDptr->maxMsgSize) {
      MSG_PRINT(F(";maxMuPrint="));
      MSG_PRINT(SDptr->maxMuPrint, DEC);
   }
   MSG_PRINT(F(";maxMsgSize="));
   MSG_PRINT(SDptr->maxMsgSize, DEC);
   if (SDptr->MuSplitThresh > 0) {
      MSG_PRINT(F(";MuSplitThresh="));
      MSG_PRINT(SDptr->MuSplitThresh, DEC);
   }
   if (SDptr->mcMinBitLen != mcMinBitLenDef) {
      MSG_PRINT(F(";mcMinBitLen="));
      MSG_PRINT(SDptr->mcMinBitLen, DEC);
   }
   if (SDptr->cMaxNumPattern != CSetDef[5]) {
      MSG_PRINT(F(";maxNumPat="));
      MSG_PRINT(SDptr->cMaxNumPattern, DEC);
   }
   if (SDptr->cMaxPulse != -maxPulse) {
      MSG_PRINT(F(";maxPulse="));
      MSG_PRINT(SDptr->cMaxPulse, DEC);
   }
   MSG_PRINT(F(";Mdebug="));
   MSG_PRINT(SDptr->MdebEnabled, DEC);
   if (SDptr->MdebEnabled) {
      MSG_PRINT(F(";MdebFifoLimit="));
      if (radionr == 1) {
         MSG_PRINT(MdebFifoLimitB, DEC);
      }
      else {
         MSG_PRINT(MdebFifoLimitA, DEC);
      }
      MSG_PRINT(F("/"));
      MSG_PRINT(FIFO_LENGTH, DEC);
   }
  }
  if (hasCC1101 == false) {
     MSG_PRINT(F(";onlyRXB=1"))
  }
   MSG_PRINTLN("");
}

//--------------------------------------------------------------------------------
inline void configCMD()
{
  bool en;
  SignalDetectorClass *SDptr;
  
  if (radionr == 0) {
    SDptr = &musterDecA;
  }
  else {
    SDptr = &musterDecB;
  }
  
  if (cmdstring.charAt(1) == 'E') {   // Enable
    en = true;
  }
  else if (cmdstring.charAt(1) == 'D') {  // Disable
    en =false;
  } else {
	unsuppCmd = true;
	return;
  }

  if (cmdstring.charAt(2) == 'S') {  	  //MS
    SDptr->MSenabled = en;
  }
  else if (cmdstring.charAt(2) == 'U') {  //MU
    SDptr->MUenabled = en;
  }
  else if (cmdstring.charAt(2) == 'C') {  //MC
    SDptr->MCenabled = en;
// }
// else if (cmdstring.charAt(2) == 'R') {  //Mreduce
//   bptr=&musterDec.MredEnabled;
  }
  else if (cmdstring.charAt(2) == 'D') {  //Mdebug
    SDptr->MdebEnabled = en;
  }
  else if (cmdstring.charAt(2) == 'L') {  //LED
    LEDenabled = en;
  }
  else if (cmdstring.charAt(2) == 'Q') {  //MSeq
    SDptr->MSeqEnabled = en;
  }
//  else if (cmdstring.charAt(2) == 'T') {  // toggleBankEnabled
//	bptr=&toggleBankEnabled;
//  }
  else {
	unsuppCmd = true;
	return;
  }

  storeFunctions(SDptr->MSenabled, SDptr->MUenabled, SDptr->MCenabled, SDptr->MredEnabled, SDptr->MdebEnabled, LEDenabled, SDptr->MSeqEnabled);
}

//--------------------------------------------------------------------------------
inline void configSET()
{ 
	char buffer[18];
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
	
	if (n == CSccmode) {		// ccmode
		if (hasCC1101 == false && (val > 1 || val < 15)) {	// bei onlyrxb ist nur 0 oder 15 zulaessig
			val = 0;
		}
		ccmode = val;
		setCCmode();
	}
	else if (n == 2) {			// mcmbl
		musterDecA.mcMinBitLen = val;
		musterDecB.mcMinBitLen = val;
	}
	else if (n == 3) {			// mscnt
		musterDecA.MsMoveCountmax = val;
		musterDecB.MsMoveCountmax = val;
	}
	else if (n == 4) {  		// fifolimitB
		MdebFifoLimitB = val;
	}
	else if (n == 7) {  		// fifolimitA
		MdebFifoLimitA = val;
	}
	else if (n == 5) {			// maxMuPrintx256
		if (val == 0) {
			val = 1;
		}
		if (val * 256 > musterDecB.maxMsgSize) {
			val = tools::EEread(CSetAddr[6]);
		}
		musterDecB.maxMuPrint = val * 256;
	}
	else if (n == 6) {			// maxMsgSizex256
		if (val <=1 ) {
			musterDecB.maxMsgSize = 254;
		}
		else if (val * 256 > defMaxMsgSize) {
			musterDecB.maxMsgSize = defMaxMsgSize;
		}
		else {
			musterDecB.maxMsgSize = val * 256;
		}
	}
	else if (n == 8) {			// maxMuPrintx256A
		if (val == 0) {
			val = 1;
		}
		if (val * 256 > musterDecA.maxMsgSize) {
			val = tools::EEread(CSetAddr[9]);
		}
		musterDecA.maxMuPrint = val * 256;
	}
	else if (n == 9) {			// maxMsgSizex256A
		if (val <=1 ) {
			musterDecA.maxMsgSize = 254;
		}
		else if (val * 256 > defMaxMsgSize) {
			musterDecA.maxMsgSize = defMaxMsgSize;
		}
		else {
			musterDecA.maxMsgSize = val * 256;
		}
	}
	else if (n == 10) {  		// onlyRXB - keine cc1101
		setHasCC1101(val);	// onlyRXB - keine cc1101
	}
	else if (n == 11) {			// maxnumpat
		musterDecA.cMaxNumPattern = val;
		musterDecB.cMaxNumPattern = val;
	}
	else if (n == 12) {			// muthreshx256
		if (val * 256 > maxPulse) {
			val = 125;
		}
		musterDecA.MuSplitThresh = val * 256;
		musterDecB.MuSplitThresh = val * 256;
	}
	else if (n == CSet16) {			// maxpulse
		if (val16 != 0) {
			musterDecA.cMaxPulse = -val16;
			musterDecB.cMaxPulse = -val16;
		}
		else {
			musterDecA.cMaxPulse = -maxPulse;
			musterDecB.cMaxPulse = -maxPulse;
		}
	}
	else if (n != CSccN) {
		unsuppCmd = true;
	}
}

//--------------------------------------------------------------------------------
uint8_t radioDetekt(bool confmode, uint8_t Dstat)
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
		MSG_PRINT(F(" Ver=0x"));
		printHex2(ver);
		if (pn == 0 && ver > 0 && ver < 0xFF) {
			MSG_PRINTLN("");
			if (confmode || (Dstat & 0x0F) == 0x0F) {
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

//--------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------
#ifdef ESP32
inline void WiFiEvent()
{
  //check if there are any new clients
  if (Server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = Server.available();
      client.flush();
      Serial.print(F("New client: "));
      Serial.println(client.remoteIP());
    } else {
      WiFiClient rejectClient = Server.available();
      rejectClient.stop();
      Serial.print(F("Reject new Client="));
      Serial.println(rejectClient.remoteIP());
      //DBG_PRINTLN("Reject new Client: ");
      //DBG_PRINTLN(rejectClient.remoteIP());
    }
  }

  if(client && !client.connected())
  {
    //DBG_PRINTLN("Client disconnected: ");
    //DBG_PRINTLN(client.remoteIP());
    client.stop();
  }
}
#endif

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

//--------------------------------------------------------------------------------
void serialEvent()
{
  while (MSG_PRINTER.available())
  {
	char inChar = (char)MSG_PRINTER.read();
   #if DEBUG_SERIAL > 1
	if ((uint8_t)inChar < 32 || (uint8_t)inChar > 126) {
		HwSerial.print(F(" 0x"));
		if ((((uint8_t)inChar) < 16)) {
			HwSerial.print(F("0"));
		}
		HwSerial.print((uint8_t)inChar, HEX);
		HwSerial.print(F(" "));
	}
	else {
		HwSerial.print(inChar);
	}
   #endif
	switch(inChar)
	{
	case '\n':
	case '\r':
	case '\0':
	//case '~':
	case '#':
	   #if DEBUG_SERIAL > 1
		HwSerial.println("");
	   #endif
	   #if DEBUG_SERIAL
		HwSerial.print(F("cmd("));
		HwSerial.print(cmdstring.length());
		HwSerial.print(F(") = "));
		HwSerial.println(cmdstring);
	   #endif
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
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

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
inline void getPing()
{
	MSG_PRINTLN(F("OK"));
	delayMicroseconds(500);
}

//--------------------------------------------------------------------------------
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
		disableReceive(false);
		RXenabled[radionr] = false;
		MSG_PRINT(F("rx"));
		MSG_WRITE('A' + radionr);
		MSG_PRINT(F("=0"));
	}
	MSG_PRINT(F(" "));
}

//--------------------------------------------------------------------------------
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
	  #if !defined(LAN_WIZ) && !defined(ESP32)	// nur bei serial wird bei XE die "Unsupported Command" Meldung aktiviert
		if (cmdstring.charAt(1) == 'E') {
			unsuppCmdEnable = true;
		}
	  #endif
		ra = 0;
		re = 4;
	}
	if (cmdstring.charAt(2) == 'W') {
		if (cmdstring.charAt(1) == 'E') {
			tools::EEwrite(addr_rxRes, 0xFF);
			MSG_PRINTLN(F("write rx=1"));
		}
		else {
			tools::EEwrite(addr_rxRes, 0xA5);	// nach einem Reset bleibt rx=0
			MSG_PRINTLN(F("write rx=0"));
		}
		tools::EEstore();
		radionr = remRadionr;
		return;
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

//--------------------------------------------------------------------------------
void setCCmode() {
  if (ccmode == 0) {	// normal OOK
    enableReceive();
    if (radionr <= 1) {
      pinAsOutput(pinSend[radionr]);
    }
  }
  else {		// mit ccFIFO
    if (radionr <= 1) {
       pinAsInput(pinSend[radionr]);
    }
    disableReceive(true);  // bei xFSK das Auslesen der Daten ueber GDO2 deaktivieren
    if (cc1101::flushrx()) {
      enableReceive();
    }
  }
}

//-- todo evtl in eigene lib
//--------------------------------------------------------------------------------
void printHex2(const uint8_t hex) {   // Todo: printf oder scanf nutzen
    if (hex < 16) {
      MSG_PRINT("0");
    }
    MSG_PRINT(hex, HEX);
}


//================================= EEProm commands ======================================

void storeFunctions(const int8_t ms, int8_t mu, int8_t mc, int8_t red, int8_t deb, int8_t led, int8_t mseq)
{
	// res mseq led deb red mc mu ms  (7 .. 0)
	mu=mu<<1;
	mc=mc<<2;
	red=red<<3;
	deb=deb<<4;
	led=led<<5;
	mseq=mseq<<6;
	
	uint8_t dat =  ms | mu | mc | red | deb | led | mseq;
	tools::EEwrite(addr_featuresB,dat);
	tools::EEstore();
}

//--------------------------------------------------------------------------------
void callGetFunctions(void)
{
	getFunctions(&musterDecA.MSenabled, &musterDecA.MUenabled, &musterDecA.MCenabled, &musterDecA.MredEnabled, &musterDecA.MdebEnabled, &LEDenabled, &musterDecA.MSeqEnabled);
	getFunctions(&musterDecB.MSenabled, &musterDecB.MUenabled, &musterDecB.MCenabled, &musterDecB.MredEnabled, &musterDecB.MdebEnabled, &LEDenabled, &musterDecB.MSeqEnabled);
	getCSvar();
}

//--------------------------------------------------------------------------------
void getFunctions(bool *ms,bool *mu,bool *mc, bool *red, bool *deb, bool *led, bool *mseq)
{
    uint8_t dat = tools::EEread(addr_featuresB);

    // res mseq led deb red mc mu ms  (7 .. 0)
    *ms=bool (dat &(1<<0));
    *mu=bool (dat &(1<<1));
    *mc=bool (dat &(1<<2));
    *red=bool (dat &(1<<3));
    *deb=bool (dat &(1<<4));
    *led=bool (dat &(1<<5));
    *mseq=bool (dat &(1<<6));
}

//--------------------------------------------------------------------------------
void getCSvar(void)
{
    uint8_t high;
    uint8_t val;
    
    MdebFifoLimitB = tools::EEread(CSetAddr[4]);
    MdebFifoLimitA = tools::EEread(CSetAddr[7]);
    musterDecB.MsMoveCountmax = tools::EEread(CSetAddr[3]);	// mscnt
    musterDecA.MsMoveCountmax = musterDecB.MsMoveCountmax;

    val = tools::EEread(CSetAddr[5]);	// maxMuPrintx256
    if (val == 0) {
       val = 1;
    }
    musterDecB.maxMuPrint = val * 256;
    val = tools::EEread(CSetAddr[8]);	// maxMuPrintx256A
    if (val == 0) {
       val = 1;
    }
    musterDecA.maxMuPrint = val * 256;

    val = tools::EEread(CSetAddr[6]);	// maxMsgSizex256
    if (val <=1 ) {
       musterDecB.maxMsgSize = 254;
    }
    else if (val * 256 > defMaxMsgSize) {
       musterDecB.maxMsgSize = defMaxMsgSize;
    }
    else {
       musterDecB.maxMsgSize = val * 256;
    }
    val = tools::EEread(CSetAddr[9]);	// maxMsgSizex256A
    if (val <=1 ) {
       musterDecA.maxMsgSize = 254;
    }
    else if (val * 256 > defMaxMsgSize) {
       musterDecA.maxMsgSize = defMaxMsgSize;
    }
    else {
       musterDecA.maxMsgSize = val * 256;
    }

    musterDecB.cMaxNumPattern = tools::EEread(CSetAddr[11]);	// maxnumpat
    musterDecA.cMaxNumPattern = musterDecB.cMaxNumPattern;

    val = tools::EEread(CSetAddr[12]);	// muthreshx256
    if (val * 256 > maxPulse) {
        val = 125;
    }
    musterDecB.MuSplitThresh = val * 256;
    musterDecA.MuSplitThresh = musterDecB.MuSplitThresh;
    
    high = tools::EEread(CSetAddr[CSet16]);	// maxpulse
    musterDecB.cMaxPulse = tools::EEread(CSetAddr[CSet16+1]) + ((high << 8) & 0xFF00);
    if (musterDecB.cMaxPulse == 0) {
       musterDecB.cMaxPulse = maxPulse;
    }
    musterDecB.cMaxPulse = -musterDecB.cMaxPulse;
    musterDecA.cMaxPulse = musterDecB.cMaxPulse;

    musterDecB.mcMinBitLen = tools::EEread(CSetAddr[2]);	// mcmbl
    if (musterDecB.mcMinBitLen == 0) {
        musterDecB.mcMinBitLen = mcMinBitLenDef;
    }
    musterDecA.mcMinBitLen = musterDecB.mcMinBitLen;
}

//--------------------------------------------------------------------------------
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

//--------------------------------------------------------------------------------
void getEthernetConfig(bool flag)
{
	uint8_t i;
	
  if (flag) {
#ifdef MAPLE_Mini
	if (tools::EEread(EE_MAC_ADDR) != mac_def[0] || tools::EEread(EE_MAC_ADDR+1) != mac_def[1] || tools::EEread(EE_MAC_ADDR+2) != mac_def[2]) {
		initEthernetConfig();
	}
	for (i = 0; i < 6; i++) {
		mac[i] = tools::EEread(EE_MAC_ADDR+i);
	}
#endif
#ifdef ESP32
	if (tools::EEread(EE_IP4_ADDR) == 0 || tools::EEread(EE_IP4_ADDR) == 255 || tools::EEread(EE_IP4_GATEWAY) == 0 || tools::EEread(EE_IP4_GATEWAY) == 255) {
		initEthernetConfig();
	}
#endif
  }
	for (i = 0; i < 4; i++) {
		ip[i] = tools::EEread(EE_IP4_ADDR+i);
		gateway[i] = tools::EEread(EE_IP4_GATEWAY+i);
		netmask[i] = tools::EEread(EE_IP4_NETMASK+i);
	}
}

//--------------------------------------------------------------------------------
void initEthernetConfig(void)
{
	for (uint8_t i = 0; i < 4; i++) {
		tools::EEwrite(EE_IP4_ADDR+i,ip_def[i]);
		tools::EEwrite(EE_IP4_GATEWAY+i,gateway_def[i]);
		tools::EEwrite(EE_IP4_NETMASK+i,netmask_def[i]);
	}
#ifdef LAN_INIT_DHCP
	tools::EEwrite(EE_IP4_ADDR+3, 0);		// Kennzeichen fuer DHCP
#endif
#ifdef MAPLE_Mini
	tools::EEwrite(EE_MAC_ADDR, mac_def[0]);
	tools::EEwrite(EE_MAC_ADDR+1, mac_def[1]);
	tools::EEwrite(EE_MAC_ADDR+2, mac_def[2]);
	uint32_t fserial = tools::flash_serial();
	tools::EEwrite(EE_MAC_ADDR+3, (uint8_t)(fserial>>16 & 0xff));
	tools::EEwrite(EE_MAC_ADDR+4, (uint8_t)(fserial>>8 & 0xff));
	tools::EEwrite(EE_MAC_ADDR+5, (uint8_t)(fserial & 0xff));
#endif
	tools::EEstore();
}

//--------------------------------------------------------------------------------
void initEEPROMconfig(void)
{
	tools::EEwrite(addr_featuresA, 0x37);    	// Init EEPROM with all flags enabled, except red, nn and toggleBank
	tools::EEwrite(addr_featuresB, 0x37);    	// Init EEPROM with all flags enabled, except red, nn and toggleBank
	
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
	tools::EEwrite(addr_res_e5, 0xFF); // reserve
	tools::EEwrite(addr_res_e6, 0xFF);
	tools::EEwrite(addr_res_e7, 0xFF);
	tools::EEwrite(addr_res_e8, 0xFF);
	tools::EEwrite(addr_rxRes, 0xFF);
	tools::EEstore();
	MSG_PRINTLN(F("Init eeprom to defaults"));
}

//--------------------------------------------------------------------------------
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
    #ifdef CMP_CC1101
       if (tools::EEread(EE_MAGIC_OFFSET) != VERSION_1) {  // ccFactoryReset nur wenn VERSION_1 nicht passt
          cc1101::ccFactoryReset(true);	// nur Bank 0
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
