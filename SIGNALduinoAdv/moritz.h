// moritz.h  8.1.2025

#ifndef _moritz_h
#define _moritz_h

#define MAX_MORITZ_MSG 30
#define max_magic0     0x5A
#define max_magic1     0x99

#define addr_max_magic0      0xBA
#define addr_max_magic1      0xBB
#define addr_max_autoAck0    0xBC
#define addr_max_autoAck1    0xBD
#define addr_max_autoAck2    0xBE

#include <Arduino.h>
#include "cc1101.h"
#include "tools.h"
#include "output.h"

extern String cmdstring;
//extern uint8_t radionr;

namespace moritz {

static uint8_t autoAckAddr[3] = {0, 0, 0};
static uint8_t fakeWallThermostatAddr[3] = {0x11, 0x11, 0x11};
static uint32_t lastTime = millis();

void moritz_handleAutoAck(uint8_t* enc);
void moritz_sendraw(uint8_t start, uint8_t end, uint8_t longPreamble);
void moritz_sendAck(uint8_t* enc);
void moritz_func(char c);
void moritz_read_AutoAckAddr();

void moritz_handleAutoAck(uint8_t* enc)
{
  uint8_t type = enc[3];
  uint8_t marcstate = cc1101::getMARCSTATE();
  
  if (marcstate != MARCSTATE_RX) {
    MSG_PRINT(F("ZERR_RX_"));
    cc1101::printHex2(marcstate);
    if (marcstate == MARCSTATE_RXFIFO_OVERFLOW) {
        MSG_PRINT(F("_OVERFL ok m="));
        cc1101::cmdStrobe(CC1101_SFRX);
    }
    cc1101::setReceiveMode();
    cc1101::printHex2(cc1101::getMARCSTATE());
    MSG_PRINTLN("");
  }
    //MSG_PRINT(F("handleAck type="));
    //cc1101::printHex2(type);
    //MSG_PRINTLN("");
  //Send acks to when required by "spec"
  if((autoAckAddr[0] != 0 || autoAckAddr[1] != 0 || autoAckAddr[2] != 0) /* auto-ack enabled */
      && (
           type == 0x03 /* type TimeInformation */
        || type == 0x30 /* type ShutterContactState */
        || type == 0x40 /* type SetTemperature */
        || type == 0x50 /* type PushButtonState */
        )
      && enc[7] == autoAckAddr[0] /* dest */
      && enc[8] == autoAckAddr[1]
      && enc[9] == autoAckAddr[2])
    moritz_sendAck(enc);

  if((fakeWallThermostatAddr[0] != 0 || fakeWallThermostatAddr[1] != 0 || fakeWallThermostatAddr[2] != 0) /* fake enabled */
      && enc[0] == 11 /* len */
      && enc[3] == 0x40 /* type SetTemperature */
      && enc[7] == fakeWallThermostatAddr[0] /* dest */
      && enc[8] == fakeWallThermostatAddr[1]
      && enc[9] == fakeWallThermostatAddr[2])
    moritz_sendAck(enc);

  return;
}
    
/* longPreamble is necessary for unsolicited messages to wakeup the receiver */
void moritz_sendraw(uint8_t start, uint8_t end, uint8_t longPreamble)
{
    uint8_t marcstate = cc1101::getMARCSTATE();
    if (marcstate != MARCSTATE_RX) { // nicht RX, error
        MSG_PRINT(F("ZERR1"));
        cc1101::printHex2(marcstate);
        if (marcstate == MARCSTATE_RXFIFO_OVERFLOW) {
            MSG_PRINT(F(" SFRX ok m="));
            cc1101::cmdStrobe(CC1101_SFRX);
            cc1101::printHex2(cc1101::getMARCSTATE());
            MSG_PRINTLN("");
        }
        else {
            MSG_PRINTLN("");
            return;
        }
    }

    // We have to keep at least 20 ms of silence between two sends (found out by trial and error)
    uint8_t n = 0;
    while(millis() - lastTime <= 20) {
      delay(1);
      n++;
    }
    if (n > 0) {
      MSG_PRINT(F("delay"));
      MSG_PRINT(n);
      MSG_PRINTLN(F("ms"));
    }

  /* Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1 (this is the case) (takes 809μs)
   * start sending - CC1101 will send preamble continuously src = enc_dstuntil TXFIFO is filled.
   * The preamble will wake up devices. See http://e2e.ti.com/support/low_power_rf/f/156/t/142864.aspx
   * It will not go into TX mode instantly if channel is not clear (see CCA_MODE), thus ccTX tries multiple times */
  if (cc1101::setTransmitMode() == false) {
    MSG_PRINT(F("ZERR2"));
    cc1101::printHex2(cc1101::getMARCSTATE());
    MSG_PRINTLN("");
    return;
  }
  
  //MSG_PRINT(F("maxSend Marc "));
  //cc1101::printHex2(cc1101::getMARCSTATE());
  //MSG_PRINTLN("");

  if (longPreamble) {
    /* Send preamble for 1 sec. Keep in mind that waiting for too long may trigger the watchdog (2 seconds on CUL) */
    for(uint8_t i=0;i<10;++i)
      delay(100); //arg is uint_8, so loop
  }

  // send
  cc1101::sendFIFO(start, end);
  
  marcstate = cc1101::getMARCSTATE();
  
  if (start == 2) {         // send or send fast
     MSG_PRINT(cmdstring);  // echo
     MSG_PRINT(F(";Marcs="));
     MSG_PRINTLN(marcstate);
  }

  if (marcstate != MARCSTATE_RX) {  // nicht RX, error
    MSG_PRINT(F("ZERR3")); 
    cc1101::printHex2(cc1101::getMARCSTATE());
    if (marcstate == MARCSTATE_TXFIFO_UNDERFLOW) {
        MSG_PRINT(" ");
        cc1101::printHex2(cc1101::readReg(CC1101_TXBYTES, CC1101_STATUS));  // Underflow and # of bytes in TXFIFO
        MSG_PRINT(F(" SFTX ok m="));
        cc1101::cmdStrobe(CC1101_SFTX);
        cc1101::setReceiveMode();
        cc1101::printHex2(cc1101::getMARCSTATE());
    }
    MSG_PRINTLN("");
  }

  lastTime = millis();
}

void moritz_sendAck(uint8_t* enc)
{
  //uint8_t ackPacket[12];
  char hexString[8];
  
  cmdstring = "0B";                    // 11 len  [0]
  sprintf(hexString, "%02X", enc[1]);  // msgcnt  [1]
  cmdstring += hexString;
  cmdstring += "0002";    // 0 flag, 2 type = Ack [2-3]
  
  sprintf(hexString, "%02X%02X%02X", enc[7], enc[8], enc[9]); // src = enc_dst [4-6]
  cmdstring += hexString;
  
  sprintf(hexString, "%02X%02X%02X", enc[4], enc[5], enc[6]); // dst = enc_src [7-9]
  cmdstring += hexString;
  
  cmdstring += "0000";     // groupid payload [10-11]

  //#ifdef ESP32
  delay(20);  /* by experiments */
  //#endif
  
  moritz_sendraw(0, 24, 0);

  //Inform FHEM that we send an autoack
  MSG_PRINT(F("z"));
  MSG_PRINTLN(cmdstring);
  cmdstring = "";
}

void moritz_func(char c)
{
    uint8_t len;
    
    if (c == 'a') {  // Auto-Ack
        MSG_PRINT(F("Za "));
        len = cmdstring.length();
        //MSG_PRINT(len);
        if (len == 8) {
            for (uint8_t i = 2; i < 8; i++) {
                if (!isHexadecimalDigit(cmdstring.charAt(i))) {
                    c = '0';
                    break;
                }
            }
        }
        else {
            c = '0';
        }
        if (c == 'a') {
            autoAckAddr[0] = tools::cmdstringPos2int(2);
            autoAckAddr[1] = tools::cmdstringPos2int(4);
            autoAckAddr[2] = tools::cmdstringPos2int(6);
            tools::EEwrite(addr_max_autoAck0, autoAckAddr[0]);
            tools::EEwrite(addr_max_autoAck1, autoAckAddr[1]);
            tools::EEwrite(addr_max_autoAck2, autoAckAddr[2]);
            tools::EEwrite(addr_max_magic0, max_magic0);
            tools::EEwrite(addr_max_magic1, max_magic1);
            tools::EEstore();
        }
        else {
            MSG_PRINTLN(F("not 6 hex characters"));
        }
    }
    if (c == 'g' || c == 'a') {
        MSG_PRINT(F("autoAckAddr:"));
        cc1101::printHex2(autoAckAddr[0]);
        cc1101::printHex2(autoAckAddr[1]);
        cc1101::printHex2(autoAckAddr[2]);
        MSG_PRINT(F(" fakeWallTAddr:"));
        cc1101::printHex2(fakeWallThermostatAddr[0]);
        cc1101::printHex2(fakeWallThermostatAddr[1]);
        cc1101::printHex2(fakeWallThermostatAddr[2]);
        MSG_PRINTLN("");
    }
    else if (c == 's' || c == 'f') {         // Send/Send fast
        len = cmdstring.length();
        /*MSG_PRINT(F("Zfunc LEN="));
        MSG_PRINT(len);
        MSG_PRINT(F(" "));
        MSG_PRINTLN(tools::cmdstringPos2int(2));
        MSG_PRINTLN(cmdstring);*/
        uint8_t hblen = (len - 4) / 2;
        if ((hblen) == tools::cmdstringPos2int(2) && hblen < MAX_MORITZ_MSG) {
            moritz_sendraw(2, len, c == 's');
            return;
        }
        MSG_PRINTLN(F("ZLENERR:"));
    }
}

void moritz_read_AutoAckAddr()
{
    autoAckAddr[0] = tools::EEread(addr_max_autoAck0);
    autoAckAddr[1] = tools::EEread(addr_max_autoAck1);
    autoAckAddr[2] = tools::EEread(addr_max_autoAck2);
}
}
#endif
