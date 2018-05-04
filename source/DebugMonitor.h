#pragma once

#ifndef DEBUGMONITOR_H
#define DEBUGMONITOR_H

#include <mbed.h>
#include <TextLCD.h>		// configure LCD at TextLCD_Config.h
#include <tuple>

class EncodedMotor; 

/** Add interface for debug using LCD2004 (tailored for GDM1718-12 group project)
 * Example: 
 * #include "DebugMonitor.h"
 * 
 * PinName knob = PA_4  // A2
 * DebugMonitor LCD(knob, motorPtr, PB_9, PB_8); 
 */
class DebugMonitor {
public:
	DebugMonitor(AnalogIn* knobPin, EncodedMotor* motorPtr, RawSerial* rawserialPtr, PinName I2C1_SDA = PB_9, PinName I2C1_SDL = PB_8,
		uint16_t lcdAddr = 0X3F, TextLCD::LCDType lcdtype = TextLCD::LCD20x4);
	void printSignal(); 

private:
	// LCD I2C Communication
	I2C i2c;
	TextLCD_I2C lcd; 

	AnalogIn* _knob;
	EncodedMotor *_motorPtr;
	RawSerial* _rawserialPtr; 
	
	std::tuple<double, unsigned long long> _speedData; 
	double _speed = 0.0f;
	long long _timeDiff = 0; 
};


#endif