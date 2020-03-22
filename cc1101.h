// cc1101.h

#ifndef _CC1101_h
#define _CC1101_h

//#ifdef defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
//#else
	//#include "WProgram.h"
//#endif
#include "output.h"
#include "tools.h"

#ifdef MAPLE_Mini
	#include <SPI.h>
#endif

extern uint16_t bankOffset;
extern uint8_t radionr;
extern String cmdstring;
extern uint8_t ccBuf[4][64];


namespace cc1101 {
	
#ifdef MAPLE_SDUINO
	#define mosiPin 28   // MOSI out
	#define misoPin 29   // MISO in
	#define sckPin  30   // SCLK out
	SPIClass SPI_2(mosiPin, misoPin, sckPin);
	const uint8_t radioCsPin[] = {31, 12, 15, 3};
#elif MAPLE_CUL
	#define mosiPin 4   // MOSI out
	#define misoPin 5   // MISO in
	#define sckPin  6   // SCLK out
	SPIClass SPI_2(mosiPin, misoPin, sckPin);
	const uint8_t radioCsPin[] = {7, 12, 15, 3};
#else
	#define csPin	SS	   // CSN  out
	#define mosiPin MOSI   // MOSI out
	#define misoPin MISO   // MISO in
	#define sckPin  SCK    // SCLK out	
#endif
	
	
	#define CC1101_CONFIG      0x80
	#define CC1101_STATUS      0xC0
	#define CC1101_WRITE_BURST 0x40
	#define CC1101_READ_BURST  0xC0
	
	#define CC1101_SYNC1       0x04
	#define CC1101_SYNC0       0x05
	#define CC1101_FREQ2       0x0D  // Frequency control word, high byte
	#define CC1101_FREQ1       0x0E  // Frequency control word, middle byte
	#define CC1101_FREQ0       0x0F  // Frequency control word, low byte
	#define CC1101_IOCFG2      0x00  // GDO2 output configuration
	#define CC1101_PKTCTRL0    0x08  // Packet config register
	#define CC1101_MDMCFG4     0x10
	#define CC1101_MDMCFG3     0x11
	#define CC1101_MDMCFG2     0x12
	
	// Multi byte memory locations
	#define CC1101_PATABLE          0x3E  // 8 byte memory
	#define CC1101_TXFIFO           0x3F
	#define CC1101_RXFIFO           0x3F

	// Status registers
	#define CC1101_RSSI      0x34 // Received signal strength indication
	#define CC1101_MARCSTATE 0x35 // Control state machine state
	#define CC1101_TXBYTES   0x3A // Underflow and # of bytes in TXFIFO
	#define CC1101_RXBYTES   0x3B // Overflow and # of bytes in RXFIFO
	
	#define MARCSTATE_SLEEP            0x00
	#define MARCSTATE_IDLE             0x01
	#define MARCSTATE_XOFF             0x02
	#define MARCSTATE_ENDCAL           0x0C
	#define MARCSTATE_RX               0x0D
	#define MARCSTATE_RX_END           0x0E
	#define MARCSTATE_RX_RST           0x0F
	#define MARCSTATE_RXFIFO_OVERFLOW  0x11
	#define MARCSTATE_TX               0x13
	#define MARCSTATE_TX_END           0x14
	#define MARCSTATE_RXTX_SWITCH      0x15
	#define MARCSTATE_TXFIFO_UNDERFLOW 0x16

	// Strobe commands
	#define CC1101_SRES     0x30  // reset
	#define CC1101_SFSTXON  0x31  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
	#define CC1101_SXOFF    0x32  // Turn off crystal oscillator
	#define CC1101_SCAL     0x33  // Calibrate frequency synthesizer and turn it off
	#define CC1101_SRX      0x34  // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
	#define CC1101_STX      0x35  // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1
	#define CC1101_SIDLE    0x36  // Exit RX / TX, turn off frequency synthesizer
	#define CC1101_SAFC     0x37  // Perform AFC adjustment of the frequency synthesizer
	#define CC1101_SFRX     0x3A  // Flush the RX FIFO buffer
	#define CC1101_SFTX     0x3B  // Flush the TX FIFO buffer.
	#define CC1101_SNOP      0x3D  // No operation. May be used to get access to the chip status byte.


/*#ifdef MAPLE_Mini
	#define wait_Miso() delayMicroseconds(10)
	#define waitV_Miso() delayMicroseconds(10)
#else
	#define wait_Miso()       while(isHigh(misoPin) ) //{ static uint8_t miso_count=255;delay(1); if(miso_count==0) return 255; miso_count--; }      // wait until SPI MISO line goes low 
	#define waitV_Miso()      while(isHigh(misoPin) ) //{ static uint8_t miso_count=255;delay(1); if(miso_count==0) return; miso_count--; }      // wait until SPI MISO line goes low 
#endif*/
#ifdef MAPLE_Mini
	#define cc1101_Select()   digitalLow(radioCsPin[radionr])          // select (SPI) CC1101
	#define cc1101_Deselect() digitalHigh(radioCsPin[radionr])
#else
	#define cc1101_Select()   digitalLow(csPin)          // select (SPI) CC1101
	#define cc1101_Deselect() digitalHigh(csPin)
#endif
	#define EE_CC1101_CFG        2
	#define EE_CC1101_CFG_SIZE   0x29
	#define EE_CC1101_PA         0x30  //  (EE_CC1101_CFG+EE_CC1101_CFG_SIZE)  // 2B
	#define EE_CC1101_PA_SIZE    8
	
	#define PATABLE_DEFAULT_433  0x84   // 5 dB default value for factory reset 433 MHz
	#define PATABLE_DEFAULT_868  0x81   // 5 dB default value for factory reset 868 MHz
	

	//------------------------------------------------------------------------------
	// Chip Status Byte
	//------------------------------------------------------------------------------

	// Bit fields in the chip status byte
	#define CC1101_STATUS_CHIP_RDYn_BM             0x80
	#define CC1101_STATUS_STATE_BM                 0x70
	#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

		// Chip states
	#define CC1101_STATE_IDLE                      0x00
	#define CC1101_STATE_RX                        0x10
	#define CC1101_STATE_TX                        0x20
	#define CC1101_STATE_FSTXON                    0x30
	#define CC1101_STATE_CALIBRATE                 0x40
	#define CC1101_STATE_SETTLING                  0x50
	#define CC1101_STATE_RX_OVERFLOW               0x60
	#define CC1101_STATE_TX_UNDERFLOW              0x70


	#ifdef ARDUINO_AVR_ICT_BOARDS_ICT_BOARDS_AVR_RADINOCC1101
	uint8_t RADINOVARIANT = 0;            // Standardwert welcher je radinoVarinat geaendert wird
	#endif
	static const uint8_t initVal[] PROGMEM = 
	{
		// IDX NAME     RESET   COMMENT
		0x0D, // 00 IOCFG2    29     GDO2 as serial output
		0x2E, // 01 IOCFG1           Tri-State
		0x2D, // 02 IOCFG0    3F     GDO0 for input
		0x07, // 03 FIFOTHR   
		0xD3, // 04 SYNC1     
		0x91, // 05 SYNC0     
		0x3D, // 06 PKTLEN    0F
		0x04, // 07 PKTCTRL1  
		0x32, // 08 PKTCTRL0  45     
		0x00, // 09 ADDR     
		0x00, // 0A CHANNR   
		0x06, // 0B FSCTRL1   0F     152kHz IF Frquency
		0x00, // 0C FSCTRL0
		0x10, // 0D FREQ2     1E     Freq
		0xB0, // 0E FREQ1     C4     
		0x71, // 0F FREQ0     EC     
		0x57, // 10 MDMCFG4   8C     bWidth 325kHz
		0xC4, // 11 MDMCFG3   22     DataRate
		0x30, // 12 MDMCFG2   02     Modulation: ASK
		0x23, // 13 MDMCFG1   22     
		0xb9, // 14 MDMCFG0   F8     ChannelSpace: 350kHz
		0x00, // 15 DEVIATN   47     
		0x07, // 16 MCSM2     07     
		0x00, // 17 MCSM1     30     Bit 3:2  RXOFF_MODE:  Select what should happen when a packet has been received: 0 = IDLE  3 =  Stay in RX ####
		0x18, // 18 MCSM0     04     Calibration: RX/TX->IDLE
		0x14, // 19 FOCCFG    36     
		0x6C, // 1A BSCFG
		0x07, // 1B AGCCTRL2  03     42 dB instead of 33dB
		0x00, // 1C AGCCTRL1  40     
		0x90, // 1D AGCCTRL0  91     4dB decision boundery
		0x87, // 1E WOREVT1
		0x6B, // 1F WOREVT0
		0xF8, // 20 WORCTRL
		0x56, // 21 FREND1
		0x11, // 22 FREND0    16     0x11 for no PA ramping
		0xE9, // 23 FSCAL3    A9    E9 ?? 
		0x2A, // 24 FSCAL2    0A    
		0x00, // 25 FSCAL1    20    19 ??
		0x1F, // 26 FSCAL0    0D     
		0x41, // 27 RCCTRL1
		0x00, // 28 RCCTRL0
	};

	void printHex2(const byte hex) {   // Todo: printf oder scanf nutzen
		if (hex < 16) {
			MSG_PRINT("0");
		}
		MSG_PRINT(hex, HEX);
	}

	uint8_t sendSPI(const uint8_t val) {					     // send byte via SPI
#ifdef MAPLE_Mini
		return SPI_2.transfer(val);
#else
		SPDR = val;                                      // transfer byte via SPI
		while (!(SPSR & _BV(SPIF)));                     // wait until SPI operation is terminated
		return SPDR;
#endif
	}
	
	uint8_t waitTo_Miso() {	// wait with timeout until MISO goes low
		uint8_t i = 255;
		while(isHigh(misoPin)) {
			delayMicroseconds(10);
			i--;
			if (i == 0) {	// timeout
				cc1101_Deselect();
				break;
			}
		}
		return i;
	}

	uint8_t cmdStrobe(const uint8_t cmd) {                  // send command strobe to the CC1101 IC via SPI
		cc1101_Select();                                // select CC1101
		//wait_Miso();                                    // wait until MISO goes low
		uint8_t ret = sendSPI(cmd);                     // send strobe command
		//wait_Miso();                                    // wait until MISO goes low
		cc1101_Deselect();                              // deselect CC1101
		return ret;					// Chip Status Byte
	}
	
	uint8_t cmdStrobeTo(const uint8_t cmd) {            // wait MISO and send command strobe to the CC1101 IC via SPI
		cc1101_Select();                                // select CC1101
		if (waitTo_Miso() == 0) {                       // wait with timeout until MISO goes low
			return false;          // timeout
		}
		sendSPI(cmd);                     // send strobe command
		//wait_Miso();                                  // wait until MISO goes low
		cc1101_Deselect();                              // deselect CC1101
		return true;
	}

	uint8_t readReg(const uint8_t regAddr, const uint8_t regType) {       // read CC1101 register via SPI
		cc1101_Select();                                // select CC1101
		//wait_Miso();                                    // wait until MISO goes low
		sendSPI(regAddr | regType);                     // send register address
		uint8_t val = sendSPI(0x00);                    // read result
		cc1101_Deselect();                              // deselect CC1101
		return val;
	}

	void writeReg(const uint8_t regAddr, const uint8_t val) {     // write single register into the CC1101 IC via SPI
		cc1101_Select();                                // select CC1101
		//waitV_Miso();                                    // wait until MISO goes low
		sendSPI(regAddr);                               // send register address
		sendSPI(val);                                   // send value
		cc1101_Deselect();                              // deselect CC1101
	}

	void readPatable(void) {
		uint8_t PatableArray[8];
		// das PatableArray wird zum zwischenspeichern der PATABLE verwendet,
		// da ich mir nicht sicher bin ob es timing maessig passt, wenn es nach jedem sendSPI(0x00) eine kurze Pause beim msgprint gibt.
		
		cc1101_Select();                                // select CC1101
		//waitV_Miso();                                    // wait until MISO goes low
		sendSPI(CC1101_PATABLE | CC1101_READ_BURST);    // send register address
		for (uint8_t i = 0; i < 8; i++) {
			PatableArray[i] = sendSPI(0x00);        // read result
		}
		cc1101_Deselect();

		for (uint8_t i = 0; i < 8; i++) {
			printHex2(PatableArray[i]);
			MSG_PRINT(" ");
		}
		MSG_PRINTLN("");
	}

	void writePatable(void) {
		cc1101_Select();                                // select CC1101
		//waitV_Miso();                                    // wait until MISO goes low
		sendSPI(CC1101_PATABLE | CC1101_WRITE_BURST);   // send register address
		for (uint8_t i = 0; i < 8; i++) {
			sendSPI(tools::EEbankRead(EE_CC1101_PA+i));                     // send value
		}
			cc1101_Deselect();
	}
	

  void readCCreg(const uint8_t reg) {   // read CC1101 register
    uint8_t var;
    uint8_t hex;
    uint8_t n;

       if (cmdstring.charAt(3) == 'n' && isHexadecimalDigit(cmdstring.charAt(4))) {   // C<reg>n<anz>  gibt anz+2 fortlaufende register zurueck
           hex = (uint8_t)cmdstring.charAt(4);
           n = tools::hex2int(hex);
           if (reg < 0x2F) {
              MSG_PRINT("C");
              printHex2(reg);
              MSG_PRINT("n");
              n += 2;
              printHex2(n);
              MSG_PRINT("=");
              for (uint8_t i = 0; i < n; i++) {
                 var = readReg(reg + i, CC1101_CONFIG);
                 printHex2(var);
              }
              MSG_PRINTLN("");
           }
       }
       else {
       if (reg < 0x3E) {
          if (reg < 0x2F) {
             var = readReg(reg, CC1101_CONFIG);
          }
          else {
             var = readReg(reg, CC1101_STATUS);
          }
          MSG_PRINT("C");
          printHex2(reg);
          MSG_PRINT(" = ");
          printHex2(var);
          MSG_PRINTLN("");
       }
       else if (reg == 0x3E) {                   // patable
          MSG_PRINT(F("C3E = "));
          readPatable();
       }
       else if (reg == 0x99) {                   // alle register
         for (uint8_t i = 0; i < 0x2f; i++) {
           if (i == 0 || i == 0x10 || i == 0x20) {
             if (i > 0) {
               MSG_PRINT(" ");
             }
             MSG_PRINT(F("ccreg "));
             printHex2(i);
             MSG_PRINT(F(": "));
           }
           var = readReg(i, CC1101_CONFIG);
           printHex2(var);
           MSG_PRINT(" ");
         }
         MSG_PRINTLN("");
       }
     }
  }

  void commandStrobes(void) {
    uint8_t hex;
    uint8_t reg;
    uint8_t val;
  
    if (isHexadecimalDigit(cmdstring.charAt(3))) {
        hex = (uint8_t)cmdstring.charAt(3);
        reg = tools::hex2int(hex) + 0x30;
        if (reg < 0x3e) {
             cc1101_Select();
             if (waitTo_Miso() == 0) {                 // wait with timeout until MISO goes low
                 MSG_PRINTLN(F("timeout!"));
                 return;
             }
             val = sendSPI(reg);                       // send strobe command
             cc1101_Deselect();
             MSG_PRINT(F("cmdStrobeReg "));
             printHex2(reg);
             MSG_PRINT(F(" chipStatus "));
             val = val >> 4;
             MSG_PRINT(val, HEX);
             if (reg != CC1101_SXOFF) {
                 delay(2);
                 val = cmdStrobe(CC1101_SNOP);        //  No operation, used to get access to the chip status byte.
                 MSG_PRINT(F(" delay2 "));
                 val = val >> 4;
                 MSG_PRINTLN(val, HEX);;
             }
             else {
                 MSG_PRINTLN("");
             }
         }
     }
  }


  void writeCCreg(uint8_t reg, uint8_t var) {    // write CC1101 register

    if (reg > 1 && reg < 0x40) {
           writeReg(reg-2, var);
           MSG_PRINT("W");
           printHex2(reg);
           printHex2(var);
           MSG_PRINTLN("");
    }
  }


void writeCCpatable(uint8_t var) {           // write 8 byte to patable (kein pa ramping)
	for (uint8_t i = 0; i < 8; i++) {
		if (i == 1) {
			tools::EEbankWrite(EE_CC1101_PA + i, var);
		} else {
			tools::EEbankWrite(EE_CC1101_PA + i, 0);
		}
	}
	#ifdef MAPLE_Mini
	tools::EEstore();
	#endif
	writePatable();
}


	void ccFactoryReset() {
		for (uint8_t i = 0; i<sizeof(initVal); i++) {
        	tools::EEbankWrite(EE_CC1101_CFG + i, pgm_read_byte(&initVal[i]));
		}
		for (uint8_t i = 0; i < 8; i++) {
			if (i == 1) {
				if (bankOffset == 0) {	// Bank 0 normalerweise 433 Mhz
					tools::EEbankWrite(EE_CC1101_PA + i, PATABLE_DEFAULT_433);
				} else {
					tools::EEbankWrite(EE_CC1101_PA + i, PATABLE_DEFAULT_868);
				}
			} else {
				tools::EEbankWrite(EE_CC1101_PA + i, 0);
			}
		}
		#ifdef MAPLE_Mini
		tools::EEstore();
		#endif
		MSG_PRINTLN(F("ccFactoryReset done"));  
	}


	uint8_t getCCVersion()
	{
		return readReg(0xF1,CC1101_CONFIG);  // Version
	}
	
	uint8_t getCCPartnum()
	{
		return readReg(0xF0,CC1101_CONFIG);  // Partnum
	}
	
	
	inline void setup()
	{
	#ifdef MAPLE_Mini
		// Setup SPI 2
		SPI_2.begin();	//Initialize the SPI_2 port.
		SPI_2.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
		pinAsOutput(radioCsPin[0]);
		digitalHigh(radioCsPin[0]);
		pinAsOutput(radioCsPin[1]);
		digitalHigh(radioCsPin[1]);
		pinAsOutput(radioCsPin[2]);
		digitalHigh(radioCsPin[2]);
		pinAsOutput(radioCsPin[3]);
		digitalHigh(radioCsPin[3]);
	#else
		pinAsOutput(sckPin);
		pinAsOutput(mosiPin);
		pinAsInput(misoPin);
		PCR = _BV(SPE) | _BV(MSTR);               // SPI speed = CLK/4
		pinAsOutput(csPin);                    // set pins for SPI communication
		digitalHigh(csPin);                 // SPI init
	#endif
		
		#ifdef PIN_MARK433
		pinAsInputPullUp(PIN_MARK433);
		#endif
		
		/*
		SPCR = ((1 << SPE) |               		// SPI Enable
		(0 << SPIE) |              		// SPI Interupt Enable
		(0 << DORD) |              		// Data Order (0:MSB first / 1:LSB first)
		(1 << MSTR) |              		// Master/Slave select
		(0 << SPR1) | (0 << SPR0) |   		// SPI Clock Rate
		(0 << CPOL) |             		// Clock Polarity (0:SCK low / 1:SCK hi when idle)
		(0 << CPHA));             		// Clock Phase (0:leading / 1:trailing edge sampling)

		SPSR = (1 << SPI2X);             		// Double Clock Rate
		*/
		pinAsInput(PIN_SEND);        // gdo0Pi, sicherheitshalber bis zum CC1101 init erstmal input   
	#ifndef MAPLE_Mini
		digitalHigh(sckPin);
		digitalLow(mosiPin);
	#else
		tools::EEbufferFill();
	#endif
	}

	uint8_t getRSSI()
	{
		return readReg(CC1101_RSSI, CC1101_STATUS);// Pruefen ob Umwandung von uint to int den richtigen Wert zurueck gibt
	}
	
	uint8_t getMARCSTATE()	// Control state machine state
	{
		return readReg(CC1101_MARCSTATE, CC1101_STATUS);// Pruefen ob Umwandung von uint to int den richtigen Wert zurueck gibt
	}
	
	uint8_t getRXBYTES()
	{
		return readReg(CC1101_RXBYTES,CC1101_STATUS);  // 
	}
	
	uint8_t getChipStatusByte()
	{
		return cmdStrobe(CC1101_SNOP);
	}
	
	
	bool readRXFIFO(uint8_t len) {
		bool dup = true;
		uint8_t rx;
		
		cc1101_Select();                                // select CC1101
		sendSPI(CC1101_RXFIFO | CC1101_READ_BURST);    // send register address
		for (uint8_t i = 0; i < len; i++) {
			rx = sendSPI(0x00);        // read result
			if (rx != ccBuf[radionr][i]) {
				dup = false;
				ccBuf[radionr][i] = rx;
			}
		}
		cc1101_Deselect();

		return dup;
	}
	
	void sendFIFO(int8_t start, uint8_t end) {
		uint8_t val;
		uint8_t i;
		
		cc1101_Select();                                // select CC1101
		//waitV_Miso();                                    // wait until MISO goes low
		sendSPI(CC1101_TXFIFO | CC1101_WRITE_BURST);   // send register address
		for (i = start; i < end; i+=2) {
			val = tools::cmdstringPos2int(i);
			//printHex2(val);
			//MSG_PRINT(F(" "));
			sendSPI(val);		// send value
		}
		cc1101_Deselect();	//Wait for sending to finish (CC1101 will go to RX state automatically

		for(i=0; i< 200;++i) 
		{
			if( readReg(CC1101_MARCSTATE, CC1101_STATUS) != MARCSTATE_TX)
				break; //neither in RX nor TX, probably some error
			delay(1);
		}
		//MSG_PRINT(F("wtx="));
		//MSG_PRINTLN(i);
		//MSG_PRINTLN("");
	}
	
	inline void setIdleMode()
	{
		cmdStrobe(CC1101_SIDLE);                             // Idle mode
		delay(1);
	}
	
	 uint8_t flushrx() {		// Flush the RX FIFO buffer
		if (cmdStrobeTo(CC1101_SIDLE) == false) {
			return false;
		}
		cmdStrobe(CC1101_SNOP);
		cmdStrobe(CC1101_SFRX);
		return true;
	}

	void setReceiveMode()
	{
		//setIdleMode();
		uint8_t maxloop = 0xff;

		while (maxloop-- &&	(cmdStrobe(CC1101_SRX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_RX) // RX enable
			delay(1);
		if (maxloop == 0 )		DBG_PRINTLN(F("CC1101: Setting RX failed"));

	}

	uint8_t setTransmitMode()
	{
		if (cmdStrobeTo(CC1101_SFTX) == false) {	// flush TX with wait MISO timeout
			DBG_PRINTLN(F("CC1101: Setting TX failed"));
			return false;
		}
		setIdleMode();
		uint8_t maxloop = 0xff;
		while (maxloop-- && (cmdStrobe(CC1101_STX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_TX)  // TX enable
			delay(1);
		if (maxloop == 0) {
			DBG_PRINTLN(F("CC1101: Setting TX failed"));
			return false;
		}
		return true;
	}
	
	bool CCreset(void) {
		cc1101_Deselect();            // some deselect and selects to init the cc1101
		delayMicroseconds(30);

		// Begin of power on reset
		cc1101_Select();
		delayMicroseconds(30);

		cc1101_Deselect();
		delayMicroseconds(45);

		cc1101_Select();
		if (waitTo_Miso() == 0) {  // wait with timeout until MISO goes low
			return false;            // timeout
		}
		sendSPI(CC1101_SRES);        // send strobe command
		
		if (waitTo_Miso() == 0) {  // wait with timeout until MISO goes low
			return false;            // timeout
		}
		cc1101_Deselect();
		
		delay(1);
		return true;
	}
	
	void CCinit_reg(void) {                              // initialize CC1101
		cc1101_Select();
		
		sendSPI(CC1101_WRITE_BURST);
		for (uint8_t i = 0; i<sizeof(initVal); i++) {              // write EEPROM value to cc1101
			sendSPI(tools::EEbankRead(EE_CC1101_CFG + i));
		}
		cc1101_Deselect();
		delayMicroseconds(10);            // ### todo: welcher Wert ist als delay sinnvoll? ###

		writePatable();                                 // write PatableArray to patable reg

		cmdStrobe(CC1101_SCAL); 
		delay(1);
	}
	
	void CCinit(void) {                              // initialize CC1101
		CCreset();
		CCinit_reg();
		//setReceiveMode();
	}

	bool regCheck()
	{
		/*
		DBG_PRINT("CC1101_PKTCTRL0="); DBG_PRINT(readReg(CC1101_PKTCTRL0, CC1101_CONFIG));
		DBG_PRINT(" vs EEPROM PKTCTRL0="); DBG_PRINTLN(initVal[CC1101_PKTCTRL0]);

		DBG_PRINT("C1100_IOCFG2="); DBG_PRINT(readReg(CC1101_IOCFG2, CC1101_CONFIG));
		DBG_PRINT(" vs EEPROM IOCFG2="); DBG_PRINTLN(initVal[CC1101_IOCFG2]);
		*/
		return (readReg(CC1101_PKTCTRL0, CC1101_CONFIG) == initVal[CC1101_PKTCTRL0]) && (readReg(CC1101_IOCFG2, CC1101_CONFIG) == initVal[CC1101_IOCFG2]);
	}

}

#endif
