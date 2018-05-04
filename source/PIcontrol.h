#pragma once

#ifndef PICONTROL_H
#define PICONTROL_H

class EncodedMotor; 

class PIcontrol {
public:
	PIcontrol() = delete; 
	PIcontrol(float Kp, float Ki);
	float compensateSignal(float error, unsigned long long int timestep);

protected: 
	float P_signal();
	float I_signal();

private:
	float _Kp = 0.0f; 
	float _Ki = 0.0f; 

	float _thisError = 0.0f; 
	float _accError_t = 0.0f; 
	unsigned long long _timestep = 0;
	float _compensateError = 0.0f; 

};

#endif // !PICONTROL_H

