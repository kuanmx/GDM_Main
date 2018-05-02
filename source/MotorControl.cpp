#include "MotorControl.h"
#include "EncodedMotor.h"
#include "PIcontrol.h"

Thread thread;

MotorControl::MotorControl(PinName motorEnablePin, PinName motorDirectionPin1, PinName motorDirectionPin2, EncodedMotor * encodedMotor, float Kp, float Ki, uint16_t ratedRPM) :
	_motorEnablePin(motorEnablePin), _motorDirectionPin1(motorDirectionPin1), _motorDirectionPin2(motorDirectionPin2), _encodedMotor(encodedMotor), 
	_picontrol(new PIcontrol(Kp, Ki)), _ratedRPM(ratedRPM)
{
	// make sure enable pin is LOW at initialize
	stop(); 

	// default direction clockwise
	setDirection(0); 
}

MotorControl::~MotorControl() {
	delete _picontrol;
}

bool MotorControl::run(double refVolt)
{
	if (_compVolt == 0) { refVolt > 0 ? setDirection(Direction::Clockwise) : setDirection(Direction::C_Clockwise); }

	_speedData = _encodedMotor->getSpeed();
	_speed = std::get<0>(_speedData);			// get speed in RPM
	_speedVolt = _speed *100 / 24;				// 24rpm rated speed
	_thisTime = std::get<1>(_speedData);		// get current timestep (us)
	
	/* Saturate error voltage to factor
	* speed up motor gradually by factor set in MotorControl.h
	* default factor is 2V / s (0.02f per 100ms, refresh rate)
	*/
	_errorVolt = refVolt * 100 - _speedVolt;
	if (_errorVolt > 5) _adjerrorVolt = 5;
	else if (_errorVolt < -5) _adjerrorVolt = -5;
	//else if (_errorVolt > 2) _adjerrorVolt = 2;
	//else if (_errorVolt < -2) _adjerrorVolt = -2;
	else if (_errorVolt < 0.4 && _errorVolt > -0.4) _adjerrorVolt = 0; 
	else _adjerrorVolt = _errorVolt; 

	// run only if latest data (data at new timestep) is provided
	if (_thisTime > _prevTime)
	{
		unsigned long timestep = _thisTime - _prevTime;	// unit us

		// PI Controller
		_compVolt += _picontrol->compensateSignal(_adjerrorVolt, timestep);
		_compVolt > 100 ? _compVolt = 100 : 0; 

		power(_compVolt/100);		// output to Motor
		_prevTime = _thisTime;		// update Timestep
	}
	return checkSteady(); 
}

/* Stop motor
* slow down motor gradually by factor set in MotorControl.h
* default factor is 2V / s (0.02f per 100ms)
* cut off at 1V (analogRead 0.1f)
*/
void MotorControl::stop()
{
	_speedData = _encodedMotor->getSpeed();
	_speed = std::get<0>(_speedData);			// get speed in RPM
	_speedVolt = _speed * 100 / 24;				// 24rpm rated speed
	_thisTime = std::get<1>(_speedData);		// get current timestep (us)
	if (_thisTime > _prevTime)
	{
		// Gradual slow procedure
		_speedVolt > 3.0f ? _compVolt -= 3.0 : _compVolt = 0;
		power(_compVolt);
		_prevTime = _thisTime; 
	}
}

void MotorControl::setDirection(MotorControl::Direction direction = MotorControl::Direction::Clockwise)
{
	if (direction == MotorControl::Direction::Clockwise) { _motorDirection1 = 0; _motorDirection2 = 1; }
	else { _motorDirection1 = 1; _motorDirection2 = 0; }
}

void MotorControl::setDirection(bool direction)
{
	direction == 0 ? setDirection(MotorControl::Direction::Clockwise) : setDirection(MotorControl::Direction::C_Clockwise); 
}

float MotorControl::readComp() { return _compVolt; }

float MotorControl::readSpeed() { return _speedVolt;  }

double MotorControl::readError() { return _errorVolt;  }

double MotorControl::readAdjError() { return _adjerrorVolt; }

void MotorControl::power(double powerIn) { _motorEnable = powerIn; }

bool MotorControl::checkSteady()
{
	return false;
}
