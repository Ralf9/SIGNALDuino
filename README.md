# SIGNALDuino uC 4.1.2 (development version) with Maple Mini, cc1101 xFSK and FIFO support

### Getting started

System to receive digital signals and provide them to other systems pro demodulatiob.

Compile it and have fun.
If you are using the Arduino IDE, you have to copy the following libs into your sketch folder:

bitstore4.h

cc1101.h

compile_config.h

output.h

signalDecoder4.cpp

signalDecoder4.h

SIGNALDuino.ino

SimpleFIFO.h

tools.h

### Using SIGNALDuino in FHEM

If you want to use the SIGNALDuino with FHEM, you can use it directly from FHEM. No neet to compile any sourcode.

### Tested microcontrollers

* MapleMini

### Signal from my device ist not detected

We have a pattern detection engine, that detect serval signal types. May not all, but most of them.

