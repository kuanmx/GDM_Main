#pragma once

#ifndef PICONTROL_H
#define PICONTROL_H

#include "../intelliSense.h"

class EncodedMotor; 

class PIcontrol {
public:
	PIcontrol() = delete; 
	PIcontrol(float Kp, float Ki);
	double compensateSignal(double error, unsigned long timestep); 

protected: 
	float P_signal();
	float I_signal();

private:
	float _Kp = 0.0f; 
	float _Ki = 0.0f; 

	float _thisError = 0.0f; 
	float _accError_t = 0.0f; 
	float _timestep = 0.0f; 
	float _compensateError = 0.0f; 

};

#endif // !PICONTROL_H

