#include "MotorControl.h"
#include "EncodedMotor.h"
#include "PIDcontrol.h"
#include "MovingAverage.h"

MovingAverage<float, 11> refSmoothing;

MotorControl::MotorControl(PinName motorEnable, PinName motorDirectionPin1, PinName motorDirectionPin2,
                           std::shared_ptr<EncodedMotor> &encodedMotor,
                           float Kp, float Ki, float Kd, float ratedRPM) :
	_motorEnable(motorEnable), _motorDirectionPin1(motorDirectionPin1), _motorDirectionPin2(motorDirectionPin2), _encodedMotor(encodedMotor),
	_piControl(std::make_unique<PIDcontrol>(Kp, Ki, Kd)), _ratedRPM(ratedRPM)
{
	stop();     // make sure enable pin is LOW at initialize
    setDirection();
}


MotorControl::~MotorControl() = default;

bool MotorControl::run()
{
    updateSpeedData();
	processInput();
    bool isSteady = false;
    // run only if latest data (data at new timeStep) is provided
	if (_thisTime > _prevTime)
	{
	    unsigned long long timeStep = _thisTime - _prevTime;	// unit us

		// PI Controller
		_compVolt += _piControl->compensateSignal(_adjErrorVolt, timeStep);
        if(_compVolt > 100 ){_compVolt = 100.0f;}           // set range of _compVolt to 0.0 - 100.0
        else if (_compVolt < 0.0f) {_compVolt = 0.0f;}


        _motorEnable.write(_compVolt/100);    // output to Motor

		_prevTime = _thisTime;		// update TimeStep
        isSteady = checkSteady();
	}

	return isSteady;
}

void MotorControl::stop()
{
    _refVolt = 0;
    run();
/*	_speedData = _encodedMotor->getSpeed();
	_speed = std::get<0>(_speedData);			// get speed in RPM
	_speedVolt = _speed * 100 / _ratedRPM;		// map rated RPM to 0 ~ 100
	_thisTime = std::get<1>(_speedData);		// get current timeStep (us)
	if (_thisTime > _prevTime)
	{
		// Gradual slow procedure
		_speedVolt > 1.0f ? _compVolt -= 1.0f : _compVolt = 0.0f;
		power(_compVolt/100);
		_prevTime = _thisTime;
	}*/
}

void MotorControl::setDirection(MotorControl::Direction direction)
{
	if (direction == MotorControl::Direction::Clockwise) { _motorDirectionPin1.write(0);  _motorDirectionPin2.write(1); }
	else { _motorDirectionPin1.write(1); _motorDirectionPin2.write(0); }
    _motorSetDirection = direction;
	// the _motorCurrentDirection hence the actual motor rotation direction
    // will be changed at MotorControl::processInput() during operation
}

void MotorControl::setRefVolt(float refVolt) {
    MotorControl::_refVolt = refVolt;
}

MotorControl::Direction MotorControl::getCurrentDirection() { return _motorCurrentDirection; }

void MotorControl::chgDirection() {
    // flip motor set direction
    if(_motorSetDirection == MotorControl::Direction::Clockwise) _motorSetDirection = MotorControl::Direction::C_Clockwise;
    else if(_motorSetDirection == MotorControl::Direction::C_Clockwise)_motorSetDirection = MotorControl::Direction::Clockwise;
}

void MotorControl::updateSpeedData() {
    _speedData = _encodedMotor->getSpeed();
    _speed = (float)std::get<0>(_speedData);	// get speed in RPM
    _speedVolt = _speed *100 / _ratedRPM;		// map rated RPM to 0 ~ 100
    _thisTime = std::get<1>(_speedData);		// get current timeStep (us)
}

void MotorControl::processInput() {
    /** Change Motor Direction
     * Check refVolt for positive (CW) / negative value (CCW)
     * Only change motor direction when _compVolt = 0 (i.e. at full stop)
     * If _setMotorDirection changed during operation, stop the motor (i.e. set refVolt = 0)
     */
    if (_motorCurrentDirection != _motorSetDirection){
        if (_compVolt == 0) {
            _motorCurrentDirection = _motorSetDirection;
            setDirection(_motorCurrentDirection);
        }
        else {_refVolt = 0; }
    }

    /** Saturate error voltage to factor
    * speed up motor gradually by factor set in MotorControl.h
    * default factor is 2V / s (0.02f per 100ms, refresh rate)
    * _errorVolt signal pull down to 0 at -0.4 ~ 0.4
    */
    _errorVolt = _refVolt * 100 - _speedVolt;

    if (_errorVolt > 2) _adjErrorVolt = 2;
    else if (_errorVolt < -2) _adjErrorVolt = -2;
    else if (_errorVolt < 0.1 && _errorVolt > -0.1) _adjErrorVolt = 0;
    else _adjErrorVolt = _errorVolt;
}

void MotorControl::setRatedRPM(float ratedRPM) { _ratedRPM = ratedRPM;}

void MotorControl::setSteadyCriteria(unsigned int continuousSteadyCriteria) {
    MotorControl::_continuousSteadyCriteria = continuousSteadyCriteria;
}

float MotorControl::readComp() { return _compVolt; }

float MotorControl::readSpeed() { return _speedVolt;  }

float MotorControl::readError() { return _errorVolt;  }

float MotorControl::readAdjError() { return _adjErrorVolt; }

float MotorControl::readRefRPM() const {
    return _refVolt*_ratedRPM;
}
bool MotorControl::checkSteady() {
    const float steadyStateCriteria = 0.01;           // steady state fluctuation within 0.001

    // increase steady count if fluctuation limit reached... reset if out of range
//    (abs(powerSmooth.getValue() - _prevPower) < steadyStateCriteria) ? steadyCount += 1 : steadyCount = 0;
    (abs(_compVolt - _prevPower) < steadyStateCriteria) ? steadyCount += 1 : steadyCount = 0;


    _prevPower = _compVolt;        // update _prevPower for next use

    if (steadyCount >= _continuousSteadyCriteria && _compVolt<99.99) return true;
    else return false;
}

unsigned int MotorControl::getSteadyCount() const {
    return steadyCount;
}


