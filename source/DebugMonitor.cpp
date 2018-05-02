#include "DebugMonitor.h"
#include "EncodedMotor.h"

DebugMonitor::DebugMonitor(PinName knobPin, EncodedMotor* motorPtr, RawSerial* rawserialPtr, PinName I2C1_SDA, PinName I2C1_SDL, uint16_t lcdAddr, TextLCD::LCDType lcdtype) :
	i2c(I2C1_SDA, I2C1_SDL), lcd(&i2c, lcdAddr << 1, lcdtype),
	_knob(knobPin), _motorPtr(motorPtr), _rawserialPtr(rawserialPtr) 
{
	lcd.cls();
	lcd.setMode(TextLCD_Base::LCDMode::DispOn);
	lcd.setBacklight(TextLCD::LCDBacklight::LightOn);

	// LCD Initialize
	lcd.printf("Initialize... ");
	wait(1.0f);
	lcd.cls();
}

void DebugMonitor::printSignal() {
	_speedData = _motorPtr->getSpeed();
	_speed = std::get<0>(_speedData);
	_timeDiff = std::get<1>(_speedData); 
	
	////// Output to LCD2004
	//// analogWrite value
	//lcd.locate(0, 0);
	//lcd.printf("PWM - %f", signal); 
	//// equivalent PWM voltage
	//lcd.locate(0, 1);
	//const float Vs_max = 10.5f; 
	//lcd.printf("Out Volt - %f", signal * Vs_max);
	//// motor RPM
	//lcd.locate(0, 2); 
	//lcd.printf("Motor RPM - %lf", _speed); 

	//// Output to Serial monitor
	_rawserialPtr->printf("---\n"); 
	_rawserialPtr->printf("refSpeed: %f\n Motor RPM: %f\n TimeDiff(us): %llu\n",
		_knob.read()*24.0f, _speed, _timeDiff); 
}
