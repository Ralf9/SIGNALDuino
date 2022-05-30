/*** copyright ***
*
* Originally initiated by D.Tostmann
* Inspired by code from TI.com AN
* License: GPL v2
*
* kaihs 2018: add support for WMBUS type C reception
* Ralf9 2022: rf_mbus.c (culfw) in mbus.h umbenannt und fuer den SIGNALDuino angepasst und erweitert
*
*/

#ifndef _MBUS_H
#define _MBUS_H

#include <Arduino.h>
#include "cc1101.h"
#include "tools.h"
#include "output.h"

#include "mbus_defs.h"
#include "mbus_packet.h"
#include "mbus_manchester.h"
#include "mbus_3outof6.h"

#define MSG_START char(0x2)
#define MSG_END char(0x3)
#define maxSendEcho 100

extern String cmdstring;
extern uint8_t radionr;
extern volatile bool blinkLED;
extern bool LEDenabled;
extern bool FSKdebug;

void mbus_init(uint8_t ccN);
void mbus_task(uint16_t Boffs_ccN);
void mbus_send(int8_t startdata);

// Buffers
uint8 MBpacket[291];
uint8 MBbytes[584];

#define WMBUS_NONE 0

#define RX_FIFO_THRESHOLD         0x07
#define RX_FIFO_START_THRESHOLD   0x00
#define RX_FIFO_SIZE              64
#define RX_OCCUPIED_FIFO          32    // Occupied space
#define RX_AVAILABLE_FIFO         32    // Free space

#define FIXED_PACKET_LENGTH       0x00
#define INFINITE_PACKET_LENGTH    0x02
#define INFINITE                  0
#define FIXED                     1
#define MAX_FIXED_LENGTH          256

#define RX_STATE_ERROR            3

typedef struct RXinfoDescr {
    uint8  lengthField;         // The L-field in the WMBUS packet
    uint16 length;              // Total number of bytes to to be read from the RX FIFO
    uint16 bytesLeft;           // Bytes left to to be read from the RX FIFO
    uint8 *pByteIndex;          // Pointer to current position in the byte array
    uint8 format;               // Infinite or fixed packet mode
    uint8 complete;             // Packet received complete
    uint8 mode;                 // S-mode or T-mode
    uint8 framemode;            // C-mode or T-mode frame
    uint8 frametype;            // Frame type A or B when in C-mode
    uint8 state;
} RXinfoDescr;


#define TX_FIFO_THRESHOLD       0x07
#define TX_OCCUPIED_FIF0        32    // Occupied space
#define TX_AVAILABLE_FIFO       32    // Free space
#define TX_FIFO_SIZE            64

#define FIXED_PACKET_LENGTH     0x00
#define INFINITE_PACKET_LENGTH  0x02
#define INFINITE                0
#define FIXED                   1
#define MAX_FIXED_LENGTH        256

#define TX_OK                   0
#define TX_LENGTH_ERROR         1
#define TX_TO_IDLE1_ERROR       2
#define TX_FLUSH_ERROR          4
#define TX_UNDERFLOW_ERROR      8
#define TX_TO_IDLE2_ERROR      16

#define TX_TOO_SHORT_OR_NOEND_ERROR 250
#define TX_NO_HEX_ERROR             251
#define TX_UNKNOWN_MODE_ERROR       252

// Struct. used to hold information used for TX
typedef struct TXinfoDescr {
    uint16 bytesLeft;           // Bytes left that are to be written to the TX FIFO
    uint8 *pByteIndex;          // Pointer to current position in the byte array
    uint8  format;              // Infinite or fixed length packet mode
    uint8  complete;            // Packet Send
} TXinfoDescr;

// Various test settings
#define CC1100_TEST2            0x2C
#define CC1100_TEST1            0x2D
#define CC1100_TEST0            0x2E

uint8_t mbus_mode = WMBUS_NONE;
RXinfoDescr RXinfo;
TXinfoDescr TXinfo;


void mbus_init(uint8_t ccN) {
    if (ccN == 11) {
        mbus_mode = WMBUS_SMODE;
    }
    else if (ccN == 12) {
        mbus_mode = WMBUS_TMODE;
    }
    cc1101::writeReg(CC1100_TEST2, 0x81);
    cc1101::writeReg(CC1100_TEST1, 0x35);
    cc1101::writeReg(CC1100_TEST0, 0x09);
    memset( &RXinfo, 0, sizeof( RXinfo ));
    MSG_PRINTLN(F("mbus_init"));
}

void mbus_init_tx(void) {

    cc1101::ccStrobe_SIDLE();  // Idle mode

    cc1101::writeReg(CC1101_FIFOTHR, TX_FIFO_THRESHOLD);

    if (mbus_mode == WMBUS_SMODE) {
      // SYNC ist bei TX und RX (7696) verschieden
      // The TX FIFO must apply the last byte of the Synchronization word
      cc1101::writeReg(CC1101_SYNC1, 0x54);
      cc1101::writeReg(CC1101_SYNC0, 0x76);
    } else { // TMODE
      // SYNC ist bei TX und RX gleich

      // Set Deviation to 50 kHz (bei RX 0x44, 38 kHz)
      cc1101::writeReg(CC1101_DEVIATN, 0x50);

      // Set data rate to 100 kbaud (bei RX 103 kbaud)
      cc1101::writeReg(CC1101_MDMCFG4, 0x5B);
      cc1101::writeReg(CC1101_MDMCFG3, 0xF8);
    }

    // Set GDO0 to be TX FIFO threshold signal
    cc1101::writeReg(CC1101_IOCFG0, 0x02);
    // Set GDO2 to be high impedance
    cc1101::writeReg(CC1101_IOCFG2, 0x2e);

    memset( &TXinfo, 0, sizeof( TXinfo ));
}

void mbus_init_tx_end(void) {

    if (mbus_mode == WMBUS_SMODE) {
      // SYNC ist bei TX und RX (7696) verschieden
      // SYNC RX
      cc1101::writeReg(CC1101_SYNC1, 0x76);
      cc1101::writeReg(CC1101_SYNC0, 0x96);
    } else { // TMODE
      // SYNC ist bei TX und RX gleich

      // Set Deviation to RX 0x44, 38 kHz)
      cc1101::writeReg(CC1101_DEVIATN, 0x44);

      // Set data rate to RX 103 kbaud
      cc1101::writeReg(CC1101_MDMCFG4, 0x5C);
      cc1101::writeReg(CC1101_MDMCFG3, 0x04);
    }
    cc1101::writeReg(CC1101_PKTLEN, 0xFF);

    // Set GDO0
    cc1101::writeReg(CC1101_IOCFG0, 0x00);
    // Set GDO2
    cc1101::writeReg(CC1101_IOCFG2, 0x06);

    memset( &RXinfo, 0, sizeof( RXinfo ));
}

static uint8_t mbus_on(uint8_t force) {
    // already in RX?
    if (!force && cc1101::getMARCSTATE() == MARCSTATE_RX)
        return 0;

    // init RX here, each time we're idle
    RXinfo.state = 0;

    cc1101::flushrx();

    // Initialize RX info variable
    RXinfo.lengthField = 0;           // Length Field in the wireless MBUS packet
    RXinfo.length      = 0;           // Total length of bytes to receive packet
    RXinfo.bytesLeft   = 0;           // Bytes left to to be read from the RX FIFO
    RXinfo.pByteIndex  = MBbytes;     // Pointer to current position in the byte array
    RXinfo.format      = INFINITE;    // Infinite or fixed packet mode
    RXinfo.complete    = FALSE;       // Packet Received
    RXinfo.mode        = mbus_mode;   // Wireless MBUS radio mode
    RXinfo.framemode   = WMBUS_NONE;  // Received frame mode (Distinguish between C- and T-mode)
    RXinfo.frametype   = 0;           // Frame A or B in C-mode

    // Set RX FIFO threshold to 4 bytes
    cc1101::writeReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
    // Set infinite length
    cc1101::writeReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

    cc1101::setReceiveMode();
    RXinfo.state = 1;

    //MSG_PRINTLN(F("mb_on"));

    return 1; // this will indicate we just have re-started RX
}


void mbus_task(uint16_t Boffs_ccN) {
    uint8_t bytesDecoded[2];
    uint8_t fixedLength;

    if (mbus_mode == WMBUS_NONE)
        return;

    switch (RXinfo.state) {
        case 0:
            mbus_on(TRUE);
            return;
        case 1:       // RX active, awaiting SYNC
            if (isHigh(pinReceive[radionr])) {
                RXinfo.state = 2;
                if (FSKdebug) {
                    MSG_PRINTLN(F("mbSyn"));
                }
            }
            break;
        // awaiting pkt len to read
        case 2:
            if (isHigh(pinSend[radionr])) {
                // Read the 3 first bytes
                cc1101::readRXFIFO(RXinfo.pByteIndex, 3, NULL, NULL);

                // - Calculate the total number of bytes to receive -
                if (RXinfo.mode == WMBUS_SMODE) {
                    // S-Mode
                    // Possible improvment: Check the return value from the deocding function,
                    // and abort RX if coding error.
                    if (manchDecode(RXinfo.pByteIndex, bytesDecoded) != MAN_DECODING_OK) {
                        RXinfo.state = 0;
                        return;
                    }
                    RXinfo.lengthField = bytesDecoded[0];
                    RXinfo.length = byteSize(1, 0, (packetSize(RXinfo.lengthField)));
                } else {
                    // In C-mode we allow receiving T-mode because they are similar. To not break any applications using T-mode,
                    // we do not include results from C-mode in T-mode.

                    // If T-mode preamble and sync is used, then the first data byte is either a valid 3outof6 byte or C-mode
                    // signaling byte. (http://www.ti.com/lit/an/swra522d/swra522d.pdf#page=6)
                    if (RXinfo.pByteIndex[0] == 0x54) {
                        RXinfo.framemode = WMBUS_CMODE;
                        // If we have determined that it is a C-mode frame, we have to determine if it is Type A or B.
                        if (RXinfo.pByteIndex[1] == 0xCD) {
                            RXinfo.frametype = WMBUS_FRAMEA;
                            // Frame format A
                            RXinfo.lengthField = RXinfo.pByteIndex[2];

                            if (RXinfo.lengthField < 9) {
                                RXinfo.state = 0;
                                return;
                            }

                            // Number of CRC bytes = 2 * ceil((L-9)/16) + 2
                            // Preamble + L-field + payload + CRC bytes
                            RXinfo.length = 2 + 1 + RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10)/16);
                        } else if (RXinfo.pByteIndex[1] == 0x3D) {
                            RXinfo.frametype = WMBUS_FRAMEB;
                            // Frame format B
                            RXinfo.lengthField = RXinfo.pByteIndex[2];

                            if (RXinfo.lengthField < 12 || RXinfo.lengthField == 128) {
                                RXinfo.state = 0;
                                return;
                            }

                            // preamble + L-field + payload
                            RXinfo.length = 2 + 1 + RXinfo.lengthField;
                        } else {
                            // Unknown type, reset.
                            RXinfo.state = 0;
                            return;
                        }
                    // T-Mode
                    // Possible improvment: Check the return value from the deocding function,
                    // and abort RX if coding error.
                    } else if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK) {
                        RXinfo.state = 0;
                        return;
                    } else {
                        RXinfo.framemode = WMBUS_TMODE;
                        RXinfo.frametype = WMBUS_FRAMEA;
                        RXinfo.lengthField = bytesDecoded[0];
                        RXinfo.length = byteSize(0, 0, (packetSize(RXinfo.lengthField)));
                    }
                }

                // check if incoming data will fit into buffer
                if (RXinfo.length > sizeof(MBbytes)) {
                    RXinfo.state = 0;
                    return;
                }

                // we got the length: now start setup chip to receive this much data
                // - Length mode -
                // Set fixed packet length mode is less than 256 bytes
                if (RXinfo.length < (MAX_FIXED_LENGTH)) {
                    cc1101::writeReg(CC1101_PKTLEN, (uint8_t)(RXinfo.length));
                    cc1101::writeReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
                    RXinfo.format = FIXED;

                // Infinite packet length mode is more than 255 bytes
                // Calculate the PKTLEN value
                } else {
                    fixedLength = RXinfo.length  % (MAX_FIXED_LENGTH);
                    cc1101::writeReg(CC1101_PKTLEN, (uint8_t)(fixedLength));
                }

                RXinfo.pByteIndex += 3;
                RXinfo.bytesLeft   = RXinfo.length - 3;

                // Set RX FIFO threshold to 32 bytes
                RXinfo.state = 3;
                cc1101::writeReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);

                //MSG_PRINT(F("mbus2 L="));
                //MSG_PRINTLN(RXinfo.bytesLeft);

                /*cc1101::printHex2(RXinfo.pByteIndex[1]);
                MSG_PRINT(" mode=");
                cc1101::printHex2(RXinfo.framemode);
                MSG_PRINT(" type=");
                cc1101::printHex2(RXinfo.frametype);
                MSG_PRINT(" L=");
                MSG_PRINTLN(RXinfo.bytesLeft);
                //MSG_PRINTLN("");
                RXinfo.state = 0;*/
            }
            break;
        // awaiting more data to be read
        case 3:
            if (isHigh(pinSend[radionr])) {
                // - Length mode -
                // Set fixed packet length mode is less than MAX_FIXED_LENGTH bytes
                if (((RXinfo.bytesLeft) < (MAX_FIXED_LENGTH )) && (RXinfo.format == INFINITE)) {
                    cc1101::writeReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
                    RXinfo.format = FIXED;
                }

                // Read out the RX FIFO
                // Do not empty the FIFO (See the CC110x or 2500 Errata Note)
                cc1101::readRXFIFO(RXinfo.pByteIndex, RX_AVAILABLE_FIFO - 1, NULL, NULL);

                if (FSKdebug) {
                    MSG_PRINT(F("L="));
                    MSG_PRINTLN(RXinfo.bytesLeft);
                }
                /*MSG_PRINT(" mode=");
                MSG_PRINT(RXinfo.framemode);
                MSG_PRINT(" type=");
                MSG_PRINTLN(RXinfo.frametype)*/
                if (RXinfo.bytesLeft < (RX_AVAILABLE_FIFO - 1)) {
                    RXinfo.state = 0;
                    return;
                }
                RXinfo.bytesLeft  -= (RX_AVAILABLE_FIFO - 1);
                RXinfo.pByteIndex += (RX_AVAILABLE_FIFO - 1);
            }
            break;
    }

    // END OF PAKET
    if (isLow(pinReceive[radionr]) && RXinfo.state>1) {
        uint8_t rssi = 0;
        uint8_t lqi = 0;
        cc1101::readRXFIFO(RXinfo.pByteIndex, (uint8_t)RXinfo.bytesLeft, &rssi, &lqi);
        RXinfo.complete = TRUE;

        // decode!
        uint16_t rxStatus = PACKET_CODING_ERROR;
        uint16_t rxLength;

        if (RXinfo.mode == WMBUS_SMODE) {
            rxStatus = decodeRXBytesSmode(MBbytes, MBpacket, packetSize(RXinfo.lengthField));
            rxLength = packetSize(MBpacket[0]);
        } else if (RXinfo.framemode == WMBUS_TMODE) {
            //for (uint16_t ii = 0; ii<RXinfo.length; ii++) {
            //    cc1101::printHex2(MBbytes[ii]);
            //}
            //MSG_PRINTLN("");
            rxStatus = decodeRXBytesTmode(MBbytes, MBpacket, packetSize(RXinfo.lengthField));
            rxLength = packetSize(MBpacket[0]);
        } else if (RXinfo.framemode == WMBUS_CMODE) {
            if (RXinfo.frametype == WMBUS_FRAMEA) {
                rxLength = RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10)/16) + 1;
                rxStatus = verifyCrcBytesCmodeA(MBbytes + 2, MBpacket, rxLength);
            } else if (RXinfo.frametype == WMBUS_FRAMEB) {
                rxLength = RXinfo.lengthField + 1;
                rxStatus = verifyCrcBytesCmodeB(MBbytes + 2, MBpacket, rxLength);
            }
        }
        if (FSKdebug) {
            MSG_PRINT(F("mb L="));
            MSG_PRINT(rxLength);
            MSG_PRINT(F(" S="));
            MSG_PRINTLN(rxStatus);
        }

        if (rxStatus == PACKET_OK) {
            if (LEDenabled) {
                blinkLED=true;
            }
            MSG_PRINT(MSG_START);
            MSG_PRINT(F("MN;D="));
            if (RXinfo.framemode == WMBUS_CMODE) {
                if (RXinfo.frametype == WMBUS_FRAMEA) {
                    MSG_PRINT(F("X")); // special marker for frame type A
                } else {
                    MSG_PRINT(F("Y")); // special marker for frame type B
                }
            }
            for (uint8_t i=0; i < rxLength; i++) {
                cc1101::printHex2(MBpacket[i] );
            }
            cc1101::printHex2(lqi);
            cc1101::printHex2(rssi);
            MSG_PRINT(F(";N="));
            MSG_PRINT(tools::EEread(Boffs_ccN));
            MSG_PRINT(F(";"));
            MSG_PRINT(MSG_END);
            MSG_PRINT("\n");
        }
        RXinfo.state = 0;
        return;
    }

    mbus_on( FALSE );
}


uint8_t txSendPacket(uint8_t* pPacket, uint8_t* pBytes, uint16_t rawlen, uint8_t cmode, bool debug) {
    uint16_t  bytesToWrite;
    uint16_t  fixedLength;
    uint8_t   txStatus;
    int8_t    retstate = 0;
    uint8_t   lastMode = WMBUS_NONE;
    uint16_t  packetLength;
    uint16_t  TXn;

    // Calculate total number of bytes in the wireless MBUS packet
    packetLength = packetSize(pPacket[0]);

    if (debug == true) {
        MSG_PRINT(F("packetlen="));
        MSG_PRINT(packetLength);
        MSG_PRINT(F(" rawlen="));
        MSG_PRINTLN(rawlen);
    }
    // Check for valid length
    if ((packetLength == 0) || (packetLength > rawlen) || (packetLength > 290))
        return TX_LENGTH_ERROR;

    mbus_init_tx();

    // Data encode packet and calculate number of bytes to transmit
    // S-mode
    if (mbus_mode == WMBUS_SMODE) {
        encodeTXBytesSmode(pBytes, pPacket, packetLength);
        TXinfo.bytesLeft = byteSize(1, 1, packetLength);
    // T-mode
    } else {
        encodeTXBytesTmode(pBytes, pPacket, packetLength);
        TXinfo.bytesLeft = byteSize(0, 1, packetLength);
    }

    if (debug == true) {
        MSG_PRINT(F("TXn="));
        MSG_PRINTLN(TXinfo.bytesLeft);
        for (uint16_t ii = 0; ii<TXinfo.bytesLeft; ii++) {
            cc1101::printHex2(pBytes[ii]);
        }
        MSG_PRINTLN("");
    }
    
    // Check TX Status
    txStatus = cc1101::ccStrobe_SNOP();
    if ( (txStatus & CC1101_STATUS_STATE_BM) != CC1101_STATE_IDLE ) {
        cc1101::ccStrobe_SIDLE();   // Idle mode
        return TX_TO_IDLE1_ERROR;
    }

    // Flush TX FIFO
    // Ensure that FIFO is empty before transmit is started
    if (cc1101::cmdStrobeTo(CC1101_SFTX) == false) {	// flush TX with wait MISO timeout
        return TX_FLUSH_ERROR;
    }

    TXn = TXinfo.bytesLeft;
    // Initialize the TXinfo struct.
    TXinfo.pByteIndex   = pBytes;
    TXinfo.complete     = FALSE;

    // Set fixed packet length mode if less than 256 bytes to transmit
    if (TXinfo.bytesLeft < (MAX_FIXED_LENGTH) ) {
        fixedLength = TXinfo.bytesLeft;
        cc1101::writeReg(CC1101_PKTLEN, (uint8_t)(TXinfo.bytesLeft));
        cc1101::writeReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
        TXinfo.format = FIXED;
    }
    // Else set infinite length mode
    else {
        fixedLength = TXinfo.bytesLeft % (MAX_FIXED_LENGTH);
        cc1101::writeReg(CC1101_PKTLEN, (uint8)fixedLength);
        cc1101::writeReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);
        TXinfo.format = INFINITE;
    }

    // Fill TX FIFO
    bytesToWrite = MIN(TX_FIFO_SIZE, TXinfo.bytesLeft);
    cc1101::WriteFifo(TXinfo.pByteIndex, bytesToWrite);
    TXinfo.pByteIndex += bytesToWrite;
    TXinfo.bytesLeft  -= bytesToWrite;

    //MSG_PRINT(F("bytesLeft2="));
    //MSG_PRINTLN(TXinfo.bytesLeft);
    // Check for completion
    if (!TXinfo.bytesLeft)
        TXinfo.complete = TRUE;

    // Strobe TX
    cc1101::cmdStrobe(CC1101_STX);

    // Wait for available space in FIFO
    //uint8_t iw = 0;
    while (!TXinfo.complete) {
        if (isLow(pinSend[radionr])) {
            //txStatus = cc1101::readReg(CC1101_TXBYTES,CC1101_STATUS);
            // Write data fragment to TX FIFO
            bytesToWrite = MIN(TX_AVAILABLE_FIFO, TXinfo.bytesLeft);
            txStatus = bytesToWrite;
            cc1101::WriteFifo(TXinfo.pByteIndex, bytesToWrite);
            //iw++;
            TXinfo.pByteIndex   += bytesToWrite;
            TXinfo.bytesLeft    -= bytesToWrite;

            if (debug == true) {
                MSG_PRINTLN(txStatus);
            }
            // Indicate complete when all bytes are written to TX FIFO
            if (!TXinfo.bytesLeft)
                TXinfo.complete = TRUE;

            // Set Fixed length mode if less than 256 left to transmit
            if ((TXinfo.bytesLeft < (MAX_FIXED_LENGTH - TX_FIFO_SIZE)) && (TXinfo.format == INFINITE)) {
                cc1101::writeReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);
                TXinfo.format = FIXED;
            }
        } else {
            // Check TX Status
            txStatus = cc1101::ccStrobe_SNOP();
            if ( (txStatus & CC1101_STATUS_STATE_BM) == CC1101_STATE_TX_UNDERFLOW ) {
                cc1101::cmdStrobe(CC1101_SFTX);
                retstate &= TX_UNDERFLOW_ERROR;
                break;
            }
        }
    }

    /*if (debug == true) {
        MSG_PRINT(F("wrfifoanz="));
        MSG_PRINTLN(iw);
    }*/

    uint8_t maxloop = 0xff;
    while (maxloop-- && (cc1101::getMARCSTATE() != MARCSTATE_IDLE))
        delay(1);
    if (maxloop == 0)
        retstate &= TX_TO_IDLE2_ERROR;

    if (retstate == 0) {
        if (cmdstring.length() <= maxSendEcho) {
            MSG_PRINT(cmdstring); // echo
        } else {
            MSG_PRINT(cmdstring.substring(0, maxSendEcho)); // echo
            MSG_PRINT(F(".. "));
        }
        MSG_PRINT(F("TXn="));
        MSG_PRINTLN(TXn);
    }
    return (retstate);
}


void mbus_send(int8_t startdata) {
    int16_t enddata;
    uint8_t val;
    uint16_t pos;
    uint8_t retstate = 0;
    uint16_t i = 0;
    bool debug = false;
    
    enddata = cmdstring.indexOf(";",startdata);   // search next   ";"
    if (enddata - startdata > 4) {
        if (cmdstring.charAt(startdata+1) == 'd') {
            debug = true;
        }
        for (pos = startdata+3; pos<enddata; pos+=2) {
            if (!isHexadecimalDigit(cmdstring.charAt(pos)) || !isHexadecimalDigit(cmdstring.charAt(pos+1))) {
                retstate = TX_NO_HEX_ERROR;
                break;
            }
            val = tools::cmdstringPos2int(pos);
            if (debug == true) {
                cc1101::printHex2(val);
            }
            MBpacket[i] = val;
            i++;
        }
        //MSG_PRINT(" radionr=");
        //MSG_PRINT(radionr);
        if (debug == true) {
            MSG_PRINTLN("");
        }
        if (retstate == 0) {
            char mode = cmdstring.charAt(startdata+2);
            if (mode == 's' || mode == 't') {
                retstate = txSendPacket(MBpacket, MBbytes, i, 0, debug);
                // re-enable RX
                mbus_init_tx_end();
            //} else if (mode == 'c') {               to do: send cmode
            //    retstate = txSendPacket(MBpacket, MBbytes, i, 1, debug);
            //    MSG_PRINTLN("cmode");
            } else {
                retstate = TX_UNKNOWN_MODE_ERROR;
            }
        }
    } else {
        retstate = TX_TOO_SHORT_OR_NOEND_ERROR;
    }
    if (retstate > 0) {
        MSG_PRINT(F("wmbus send failed! ret="));
        MSG_PRINTLN(retstate);
    }
}

#endif
