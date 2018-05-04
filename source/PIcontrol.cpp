#include "mbed.h"
#include "PIcontrol.h"

PIcontrol::PIcontrol(float Kp, float Ki)
	: _Kp(Kp), _Ki(Ki)
{
}

float PIcontrol::compensateSignal(float error, unsigned long long int timestep)
{
	_compensateError = 0.0f; 

	_thisError = error;
	_timestep = timestep;

	_compensateError += P_signal(); 
	_compensateError += I_signal();

	return _compensateError;
}

float PIcontrol::P_signal()
{
	return _Kp * _thisError; 
}

float PIcontrol::I_signal()
{
	_accError_t += _thisError * _timestep /1000000;
	return _Ki * (_accError_t); 
}
