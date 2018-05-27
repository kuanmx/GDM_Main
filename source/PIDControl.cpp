#include "mbed.h"
#include "PIDControl.h"
#include "MovingAverage.h"


MovingAverage<float, 3> dSmoothing;

PIDControl::PIDControl(float Kp, float Ki, float Kd)
	: _Kp(Kp), _Ki(Ki), _Kd(Kd)
{
}

float PIDControl::compensateSignal(float error, unsigned long long int timestep)
{
	_compensateError = 0.0f; 

	_thisError = error;
	_timestep = timestep;

	auto pError = error * _Kp;
	_accError += error *  timestep / 1000000;
	auto iError = _accError * _Ki;
	auto dError = (error - _prevError) * 1000000 / timestep;
	dSmoothing.AddData(dError);
	_compensateError += pError + _accError + dSmoothing.getValue();

	return _compensateError;
}

float PIDControl::P_signal()
{
	return _Kp * _thisError; 
}

float PIDControl::I_signal()
{
	_accError += _thisError * _timestep /1000000;
	return _Ki * (_accError);
}
