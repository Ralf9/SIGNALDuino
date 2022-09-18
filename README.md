# SIGNALDuino uC v4.2.2 (development version) for Maple Mini and ESP32 with multi cc1101 and xFSK support

### Getting started

System to receive digital signals and provide them to other systems pro demodulatiob.

Firmware files  are in https://github.com/Ralf9/SIGNALDuino/releases

Compile it or use the and have fun.  
- If you are using the Arduino IDE, you have to copy the folder "SIGNALduinoAdv" with its contents into your sketch folder and edit the file compile_config.h 
 - If you use LAN, you need the Ethernet Library
 - If you use the ESP32, you need the WifiManager by tablatronix/tzapu
The ESP32 board version 1.06 is the last version with the ESP-IDF v3.3.x, there may be WiFi problems with the ESP-IDF v4.4
- If you are using platformio, you have to copy the files "extra_script.py", "platformio.ini" and the folder "SIGNALduinoAdv" with its contents into your project folder. 

### Using SIGNALDuino in FHEM

If you want to use the SIGNALDuino with FHEM, you can use it directly from FHEM. No need to compile any sourecode.
You find more Information here: https://wiki.fhem.de/wiki/Maple-SignalDuino  
https://forum.fhem.de/index.php/topic,106278.0.html

### Tested microcontrollers

* STM32 F103CBT6 (Maple Mini, bootloader v2.0)
* ESP32

### Signal from my device ist not detected

We have a pattern detection engine, that detect serval signal types. May not all, but most of them.

