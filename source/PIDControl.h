#pragma once

#ifndef PICONTROL_H
#define PICONTROL_H

class EncodedMotor; 

class PIDControl {
public:
	PIDControl() = delete;
	PIDControl(float Kp, float Ki, float Kd);
	float compensateSignal(float error, unsigned long long int timestep);

protected: 
	float P_signal();
	float I_signal();

private:
	float _Kp = 0.0f; 
	float _Ki = 0.0f;
	float _Kd = 0.0f;

	float _thisError = 0.0f;
	float _prevError = 0.0f;
	float _accError = 0.0f;
	unsigned long long _timestep = 0;
	float _compensateError = 0.0f; 

};

#endif // !PICONTROL_H

