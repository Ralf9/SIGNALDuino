#pragma once

#ifndef SimpleFIFO_h
#define SimpleFIFO_h
#include <Arduino.h>
/*
||
|| @file 		SimpleFIFO.h
|| @version 	1.2
|| @author 	Alexander Brevig
|| @contact 	alexanderbrevig@gmail.com
||
|| @description
|| | A simple FIFO class, mostly for primitive types but can be used with classes if assignment to int is allowed
|| | This FIFO is not dynamic, so be sure to choose an appropriate size for it
|| #
||
|| @license
|| | Copyright (c) 2010 Alexander Brevig
|| | This library is free software; you can redistribute it and/or
|| | modify it under the terms of the GNU Lesser General Public
|| | License as published by the Free Software Foundation; version
|| | 2.1 of the License.
|| |
|| | This library is distributed in the hope that it will be useful,
|| | but WITHOUT ANY WARRANTY; without even the implied warranty of
|| | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
|| | Lesser General Public License for more details.
|| |
|| | You should have received a copy of the GNU Lesser General Public
|| | License along with this library; if not, write to the Free Software
|| | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
|| #
||
*/

#define FIFO_LENGTH            200

class SimpleFIFO {
public:

	SimpleFIFO(uint8_t rawsize);

	int16_t dequeue();				//get next element
	void IRAM_ATTR enqueue(int16_t element );	//add an element
	void flush();				//[1.1] reset to default state 
	
	uint8_t count() { return numberOfElements; }

private:
  uint8_t size;
  volatile uint8_t numberOfElements = 0;
  volatile uint8_t nextIn = 0;
  volatile uint8_t nextOut = 0;
  volatile int16_t rawFifo[FIFO_LENGTH];
};

SimpleFIFO::SimpleFIFO(uint8_t rawsize)
{
	size = rawsize; // configured size
	flush();
}

void IRAM_ATTR SimpleFIFO::enqueue(int16_t element) {
	if ( numberOfElements >=  size) { return; }
	numberOfElements++;
	nextIn %= size;
	rawFifo[nextIn] = element;
	nextIn++;
	return;
}

int16_t SimpleFIFO::dequeue() {
	numberOfElements--;
	nextOut %= size;
	return rawFifo[nextOut++];
}

void SimpleFIFO::flush() {
	nextIn = nextOut = numberOfElements = 0;
}

#endif
