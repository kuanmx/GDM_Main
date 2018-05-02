//
// Created by poh14 on 3/24/2018.
//

#ifndef MBEDCLI2_ENCODEDMOTOR_H
#define MBEDCLI2_ENCODEDMOTOR_H

#include "../intelliSense.h"
#include <mbed.h>
#include <tuple>

enum class EncodeType:uint8_t {
  X1 = 1,
  X2 = 2,
  X4 = 4
};
class EncodedMotor {
public:
    EncodedMotor() = delete;
    EncodedMotor(PinName encoderA, PinName encoderB, unsigned int pulsePerRotation,
            float samplingRate = 1.0f, EncodeType encodeType = EncodeType::X1);

    void Stop();
    std::tuple<double, unsigned long long> getSpeed() const;


private:
    //Methods
    void incrementPulse(){
        _pulseBuffer++;
    }

    void saveData();

    //Data
    InterruptIn _encoderAInterrupt, _encoderBInterrupt;
    const unsigned int _pulsePerRotation;
    float _samplingRate;
	float _samplingPeriod;
    EncodeType _encodeType;

    double _speed = 0;
    unsigned long _pulseBuffer = 0;
    unsigned long long _previousSaveTime = 0;
    unsigned long long _previousReadTime = 0;
    Timer timer;
    Ticker ticker;

};

#endif //MBEDCLI2_ENCODEDMOTOR_H
