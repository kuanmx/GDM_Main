#pragma once

#ifndef  MOTORCONTROL_H
#define MOTORCONTROL_H

#include <mbed.h>
#include <tuple>

class EncodedMotor; 
class PIcontrol;

/** Motor Controller with PI Control
* run(*) and stop() method must be placed in continuous loop
*/
class MotorControl {
public:
	MotorControl() = delete; 
	MotorControl(PinName motorEnable, PinName motorDirectionPin1, PinName motorDirectionPin2, EncodedMotor* encodedMotor, 
		float Kp = 1, float Ki = 0, uint16_t ratedRPM = 24);
	~MotorControl();
	enum class Direction {
		Clockwise,
		C_Clockwise
	};

	/** start motor
	* start motor gradually by factor set in MotorControl.h
	* default factor is 2V / s
	* max volt per step at 1.2V (0.1f)
	* start from 1V (PwmOut 0.1f)
	* input reference power for power (MotorControl will adjust motor to actual power)
	* return true if in steady
	*/
	bool run(double refPower);	
	
	/** Stop motor
	* slow down motor gradually by factor set in MotorControl.h
	* default factor is 2V / s (0.02f per 100ms)
	* cut off at 1V (analogRead 0.1f)
	*/
	void stop(); 
	void setDirection(Direction direction);		
	void setDirection(bool direction);			// 0: clockwise; 1: counter-clockwise
	float readComp(); 
	float readSpeed(); 
	double readError(); 
	double readAdjError(); 

	// Shorthand for MotorControl::run(float powerIn)
	/*MotorControl & MotorControl::operator=(float powerIn)
	{
		run(powerIn);
		return *this;
	}*/
protected:

private:
	// define PinName
	PinName _motorEnablePin; 
	PinName _motorDirectionPin1; 
	PinName _motorDirectionPin2;
	EncodedMotor* _encodedMotor;
	PIcontrol* _picontrol = nullptr;

	// define Port
	PwmOut _motorEnable = _motorEnablePin; 
	DigitalOut _motorDirection1 = _motorDirectionPin1;
	DigitalOut _motorDirection2 = _motorDirectionPin2;

	// define Constant
	uint16_t _ratedRPM = 0;
	double _speedVolt = 0.0f;		// step up output by 100 for comparison control
	double _adjerrorVolt = 0.0f;	// step up output by 100 for comparison control
	double _errorVolt = 0.0f;		// step up output by 100 for comparison control
	double _compVolt = 0.0f;		// step up output by 100 for comparison control
	std::tuple<double, unsigned long long> _speedData;
	double _speed = 0.0f; 
	long long _thisTime = 0; 
	long long _prevTime = 0; 

	const unsigned int factor = 2;  // Volt per 0.1s

	// define funtion and object
	void power(double powerIn);
	Ticker ticker;
	bool checkSteady(); 

}; 
#endif // ! MOTORCONTROL_H