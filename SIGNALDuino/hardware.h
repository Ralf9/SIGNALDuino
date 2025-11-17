/*
 * hardware.h
 *
 *  Created on: 23.09.2024
 *      Author: loetmeister.de
 */

#ifndef _hardware_h
#define _hardware_h


#if defined (ARDUINO_ARCH_RP2040)
  #include <malloc.h>
  // user "internal" EEPROM (simulated on flash memory)
  #include <EEPROM.h>
  #define EEPROM_RPI_simulated
  #define EEPROM_RPI_size 1024
  
  #ifndef NOT_A_PIN
  #define NOT_A_PIN 0xFF
  #endif
#else
  #include <EEPROM.h>
#endif

/* watchdog macros, config, reboot and reset for SIGNALDuino*/
#if defined (__AVR__) and defined(WATCHDOG)
  #include "avr/wdt.h"
  #define RESET_HARDWARE() while(1){}
  #define ENABLE_WATCHDOG() wdt_enable(WDTO_2S)
  #define DISABLE_WATCHDOG() wdt_disable()
  #define RESET_WATCHDOG() wdt_reset()
  #define WATCHDOG_CAUSED_RESET() MCUSR & (1 << WDRF)

#elif defined (ARDUINO_ARCH_RP2040) and defined(WATCHDOG)
  #include <hardware/watchdog.h>
  #define ENABLE_WATCHDOG() watchdog_enable(2000, 0)
  #define DISABLE_WATCHDOG() watchdog_disable()
  #define RESET_WATCHDOG() watchdog_update()
//watchdog_reboot(0, 0, 10);  // watchdog fire after 10us and busy waits // or watchdog_reboot(0, SRAM_END, 0) ??
  #define RESET_HARDWARE() watchdog_reboot(0, SRAM_END, 10);\
    for (;;) { }
  #define WATCHDOG_CAUSED_RESET() watchdog_caused_reboot()
#endif


#endif /* _hardware_h */
