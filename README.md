# SIGNALDuino uC 3.3.5 (development version) with cc1101 xFSK and FIFO support

### Getting started

System to receive digital signals and provide them to other systems pro demodulatiob.
Compile it and have fun. If you are using the Arduino IDE, you have to copy the following libs into your sketch folder:

bitstore.h

cc1101.h

compile_config.h

FastDelegate.h

output.h

signalDecoder.cpp

signalDecoder.h

SIGNALDuino.ino

SimpleFIFO.h

in addition, Paul Stoffregen’s "TimerOne" Library is required

### Using SIGNALDuino in FHEM

If you want to use the SIGNALDuino with FHEM, you can use it directly from FHEM. No need to compile any sourcode.
You find more Information here:

https://forum.fhem.de/index.php/topic,82379.msg1010643.html#msg1010643

https://forum.fhem.de/index.php/topic,111653.msg1058900.html#msg1058900


### supported microcontrollers

* Aduino Nano
* Arduino Pro Mini
* BusWare CUL V3 (ATmega32U4)
* RadinoCC1101 (not yet tested)

### Signal from my device ist not detected

We have a pattern detection engine, that detect serval signal types. May not all, but most of them.
