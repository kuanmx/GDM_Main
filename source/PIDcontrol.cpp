#include "mbed.h"
#include "PIDcontrol.h"

PIDcontrol::PIDcontrol(float Kp, float Ki, float Kd)
	: _Kp(Kp), _Ki(Ki), _Kd(Kd)
{
}

float PIDcontrol::compensateSignal(float error, unsigned long long int timeStep)
{
	_compensateError = 0.0f; 

	_thisError = error;
	_timeStep = timeStep;

	_compensateError += P_signal(); 
	_compensateError += I_signal();
	_compensateError += D_signal();

    _prevError = _thisError;    // update Previous Error for next calculation

    return _compensateError;
}

float PIDcontrol::P_signal()
{
	return _Kp * _thisError; 
}

float PIDcontrol::I_signal()
{
	_accError_t += _thisError * _timeStep /1000000;  // _timeStep in microseconds
	return _Ki * (_accError_t);
}

float PIDcontrol::D_signal() {
    float signal = (_thisError - _prevError)/_timeStep;
    return 0;
}
