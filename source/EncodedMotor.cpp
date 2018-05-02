//
// Created by poh14 on 3/24/2018.
//

#include "EncodedMotor.h"
//#include <TextLCD.h>
//#include <functional>


EncodedMotor::EncodedMotor(PinName encoderA, PinName encoderB, unsigned int pulsePerRotation,
        float samplingRate, EncodeType encodeType) :_encoderAInterrupt(encoderA), _encoderBInterrupt(encoderB),
                                                    _pulsePerRotation(pulsePerRotation),
                                                    _samplingRate(samplingRate), _samplingPeriod(1/samplingRate),
                                                    _encodeType(encodeType)
{
    switch(_encodeType){
    case EncodeType::X1:
        _encoderAInterrupt.rise(callback(this, &EncodedMotor::incrementPulse));
        break;
    case EncodeType::X2:
        _encoderAInterrupt.rise(callback(this, &EncodedMotor::incrementPulse));
        _encoderAInterrupt.fall(callback(this, &EncodedMotor::incrementPulse));
        break;
    case EncodeType::X4:
        _encoderAInterrupt.rise(callback(this, &EncodedMotor::incrementPulse));
        _encoderAInterrupt.fall(callback(this, &EncodedMotor::incrementPulse));
        _encoderBInterrupt.rise(callback(this, &EncodedMotor::incrementPulse));
        _encoderBInterrupt.fall(callback(this, &EncodedMotor::incrementPulse));
        break;
    }

    timer.start();
    ticker.attach(callback(this, &EncodedMotor::saveData), _samplingPeriod);

}
void EncodedMotor::saveData()
{
    unsigned long long currentTime = timer.read_high_resolution_us();
    unsigned long long time_us =currentTime - _previousSaveTime;
    _previousSaveTime = currentTime;
    double time_m = time_us/60/1000000.0;
    float rotation = (_pulseBuffer/(float)_pulsePerRotation); //rotation amount (rev)
    _pulseBuffer = 0;
    _speed = rotation/time_m; //rpm
}
void EncodedMotor::Stop()
{
    switch(_encodeType){
    case EncodeType::X1:
        _encoderAInterrupt.rise(NULL);
        break;
    case EncodeType::X2:
        _encoderAInterrupt.rise(NULL);
        _encoderAInterrupt.fall(NULL);
        break;
    case EncodeType::X4:
        _encoderAInterrupt.rise(NULL);
        _encoderAInterrupt.fall(NULL);
        _encoderBInterrupt.rise(NULL);
        _encoderBInterrupt.fall(NULL);
        break;
    }
    ticker.detach();

}
std::tuple<double, unsigned long long> EncodedMotor::getSpeed() const
{
    unsigned long long timeDiff_us = _previousSaveTime - _previousReadTime;
    return std::make_tuple(_speed, timeDiff_us);
}
