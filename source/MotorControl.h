#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <mbed.h>
#include <tuple>
#include <vector>

class EncodedMotor; 
class PIcontrol;

/** Motor Controller with PI Control
* run(*) and stop() method must be placed in continuous loop
*/
class MotorControl {
public:
	MotorControl() = delete; 
	MotorControl(RawSerial* pc, PwmOut* motorEnablePwmPin, DigitalOut* motorDirectionPin1, DigitalOut* motorDirectionPin2, EncodedMotor* encodedMotor,
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
	bool run(float refPower);
	
	/** Stop motor
	* slow down motor gradually by factor set in MotorControl.h
	* default factor is 2V / s (0.02f per 100ms)
	* cut off at 1V (analogRead 0.1f)
	*/
	void stop();

	void setDirection(Direction direction);		
	void setDirection(bool direction);			// 0: clockwise; 1: counter-clockwise

    /** Set Motor Output RPM
     * @param ratedRPM
     * set motor output shaft rated RPM
     * default is 24 RPM
     */
    void setRatedRPM(unsigned int ratedRPM = 24);

	float readComp(); 
	float readSpeed(); 
	float readError();
	float readAdjError();

	// Shorthand for MotorControl::run(float powerIn)
	/*MotorControl & MotorControl::operator=(float powerIn)
	{
		run(powerIn);
		return *this;
	}*/
protected:

private:
	// define Port
    RawSerial* _pc;
    PwmOut* _motorEnable;
    DigitalOut* _motorDirectionPin1;
    DigitalOut* _motorDirectionPin2;
	EncodedMotor* _encodedMotor;
	PIcontrol* _picontrol = nullptr;

	// define Constant
	unsigned int _ratedRPM = 0;
	float _speedVolt = 0.0f;		// step up output by 100 for comparison control
	float _adjErrorVolt = 0.0f;	// step up output by 100 for comparison control
	float _errorVolt = 0.0f;		// step up output by 100 for comparison control
	float _compVolt = 0.0f;		// step up output by 100 for comparison control
	std::tuple<double, unsigned long long> _speedData;
	float _speed = 0.0f;
	unsigned long long _thisTime = 0;
	unsigned long long _prevTime = 0;

	const unsigned int factor = 2;  // Volt per 0.1s

	// define function and object
	void power(float powerIn);
	bool checkSteady();
	float refSpeedSmoothing(float input);

}; 
#endif // ! MOTORCONTROL_H