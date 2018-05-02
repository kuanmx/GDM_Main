#include "mbed.h"
#include "PIcontrol.h"

PIcontrol::PIcontrol(float Kp, float Ki)
	: _Kp(Kp), _Ki(Ki)
{
}

double PIcontrol::compensateSignal(double error, unsigned long timestep) 
{
	_compensateError = 0.0f; 

	_thisError = error; 
	_timestep = (double) timestep / 1000000;

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
	_accError_t += _thisError * _timestep; 
	return _Ki * (_accError_t); 
}
