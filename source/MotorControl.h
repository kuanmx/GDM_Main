#pragma once

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <mbed.h>
#include <tuple>
#include <memory>

class EncodedMotor; 
class PIDcontrol;

/** Motor Controller with PI Control
* run(*) and stop() method must be placed in continuous loop
*/
class MotorControl {
public:
	MotorControl() = delete;
	MotorControl(PinName motorEnablePwmPin, PinName motorDirectionPin1, PinName motorDirectionPin2,
                 std::shared_ptr<EncodedMotor> &encodedMotor,
                 float Kp = 1, float Ki = 0, float Kd = 0, float ratedRPM = 24);
	~MotorControl();
	enum class Direction {Clockwise = 0, C_Clockwise};

	/** start motor
	* start motor gradually by factor set in MotorControl.h
	* default factor is 2V / s
	* max volt per step at 1.2V (0.1f)
	* start from 1V (PwmOut 0.1f)
	* input reference power for power (MotorControl will adjust motor to actual power)
	* return true if in steady
	*/
    bool run();
	
	/** Stop motor
	* slow down motor gradually by factor set in MotorControl.h
	* default factor is 2V / s (0.02f per 100ms)
	* cut off at 1V (analogRead 0.1f)
	*/
	void stop();

	/** Flip direction of motor
	 * Motor rotating direction will not change immediately
	 * If pressed twice, the set direction will be flipped twice, hence rotating direction will not change
	 * Default direction is clockwise
	 */
	void chgDirection();

	/** Get the current direction of motor
	 * Not necessary the direction set for direction of motor
	 * (i.e. pending changes)
	 * @return motorDirection
	 */
	Direction getCurrentDirection();

    /** Set Motor Output RPM
     * @param ratedRPM
     * set motor output shaft rated RPM
     * default is 24 RPM
     */
    void setRatedRPM(float ratedRPM = 24);

    /** Set continuous steady criteria met (within 0.01 out of 1) to declare motor in steady state
     * @param continuousSteadyCriteria
     * default is 5
     */
    void setSteadyCriteria(unsigned int continuousSteadyCriteria = 5);
    void setRefVolt(float _refVolt);

	float readComp();       // return compensate voltage
	float readSpeed();      // return speed voltage
	float readError();      // return error voltage
	float readAdjError();   // return adjusted error voltage
    float readRefRPM() const;
    unsigned int getSteadyCount() const;

protected:

private:
	// define Port
    PwmOut _motorEnable;
    DigitalOut _motorDirectionPin1;
    DigitalOut _motorDirectionPin2;
	std::shared_ptr<EncodedMotor> _encodedMotor;
	std::unique_ptr<PIDcontrol> _piControl;

	// define Constant
	float _ratedRPM = 0;
	float _speedVolt = 0.0f;	    // step up output by 100 for comparison control
	float _adjErrorVolt = 0.0f;	    // step up output by 100 for comparison control
	float _errorVolt = 0.0f;	    // step up output by 100 for comparison control
	float _compVolt = 0.0f;		    // step up output by 100 for comparison control
    float _refVolt = 0.0f;          // mapped to -1.0 to 1.0
	std::tuple<double, unsigned long long> _speedData;
	float _speed = 0.0f;
	unsigned long long _thisTime = 0;
	unsigned long long _prevTime = 0;
	float _prevPower = 0.0f;
	// record previous write power
	unsigned int steadyCount = 0;
    // count number of continued steady state
    unsigned int _continuousSteadyCriteria = 5;    // set continuous steady criteria met before steady state is declared

    const unsigned int factor = 2;  	// Volt per 0.1s

	// define function and object
	void updateSpeedData();             // Function to handle updating current speed data
	void processInput();                // Function to handle input signal processing
    void setDirection(Direction direction = MotorControl::Direction::Clockwise);     // Private function to change direction of motor directly without safeguard
	bool checkSteady();                 // Function to check if motor reach steady state
    Direction _motorCurrentDirection = Direction::Clockwise;   // Current Direction of Motor
    Direction _motorSetDirection = Direction::Clockwise;       // Direction Set for Motor (i.e. pending changes)

}; 
#endif // ! MOTORCONTROL_H