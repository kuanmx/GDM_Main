#pragma once

#ifndef PICONTROL_H
#define PICONTROL_H

class EncodedMotor; 

class PIDcontrol {
public:
	PIDcontrol() = delete;
	PIDcontrol(float Kp, float Ki, float Kd);
	float compensateSignal(float error, unsigned long long int timeStep);

protected: 
	float P_signal();
	float I_signal();
	float D_signal();

private:
	float _Kp = 0.0f; 
	float _Ki = 0.0f;
	float _Kd = 0.0f;

	float _thisError = 0.0f;
	float _prevError = 0.0f;
	float _accError_t = 0.0f; 
	unsigned long long _timeStep = 0;
	float _compensateError = 0.0f; 

};

#endif // !PICONTROL_H

