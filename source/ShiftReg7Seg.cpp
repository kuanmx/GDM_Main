//
// Created by poh14 on 3/29/2018.
//

#include "ShiftReg7Seg.h"

//ShiftReg7Seg::ShiftReg7Seg(SPI *spiObj, PinName latchPin, unsigned int numberOfDisplay, PinName MRPin)
//        :_spiPtr(spiObj), _latchPin(latchPin), _MRPin(MRPin), _numberOfDisplay(numberOfDisplay)
//{
//    _numberOfDP = _numberOfDisplay -1;
//    _MRPin = 1;
//}
ShiftReg7Seg::ShiftReg7Seg(PinName MOSI_Pin, PinName MISO_Pin, PinName SCK_Pin, PinName latchPin,
        unsigned int numberOfDisplay, PinName MRPin) : _spiPtr(std::make_unique<SPI>(MOSI_Pin, MISO_Pin, SCK_Pin)),
                                                       _latchPin(latchPin), _MRPin(MRPin), _numberOfDisplay(numberOfDisplay)
{
    _numberOfDP = _numberOfDisplay - 1;
    _MRPin = 1;
    _spiPtr->frequency(40000000);
}

unsigned int ShiftReg7Seg::getNumberOfDisplay() const
{
    return _numberOfDisplay;
}
void ShiftReg7Seg::setNumberOfDisplay(unsigned int _numberOfDisplay)
{
    ShiftReg7Seg::_numberOfDisplay = _numberOfDisplay;
}
int ShiftReg7Seg::getNumberOfDP() const
{
    return _numberOfDP;
}
void ShiftReg7Seg::setNumberOfDP(int _numberOfDP)
{
    ShiftReg7Seg::_numberOfDP = _numberOfDP;
}
std::vector<uint8_t> ShiftReg7Seg::display(double value)
{
    double logVal = log10(value);
    logVal>=0 ? logVal : logVal = 0;
    auto numberOfDigits = static_cast<unsigned> (floor(logVal)+1);
    if (numberOfDigits>_numberOfDisplay) {
        std::vector<uint8_t> errorArray(_numberOfDisplay, characterMap[10]);
        return displayDigits(errorArray);
    }
    else {
        std::vector<uint8_t> numArray(_numberOfDisplay);
        for (size_t i = 0; i<_numberOfDisplay; i++) {
            int power = i+1-numberOfDigits;
            double scaler = pow(10, power);
            double scaledVal = value*scaler;
            auto flooredScaledVal = static_cast<unsigned>(floor(scaledVal));
            unsigned digit = flooredScaledVal%10;
            numArray[i] = characterMap[digit];
        }
        numArray[numberOfDigits-1] = static_cast<uint8_t>(numArray[numberOfDigits-1] | 0b00000001);

        return displayDigits(numArray);
    }
}

std::vector<uint8_t> ShiftReg7Seg::displayDigits(std::vector<uint8_t>& digitArray)
{
    std::vector<uint8_t> returnArray;
    returnArray.reserve(digitArray.size());
    for (auto iter = std::rbegin(digitArray);iter!=std::rend(digitArray);iter++) {
        _latchPin = 0;
        _spiPtr->write(*iter);
        returnArray.push_back(static_cast<uint8_t>(_spiPtr->write(*iter)));
        _latchPin = 1;
    }
    return returnArray;

}
void ShiftReg7Seg::clearAll()
{
    _MRPin = 0;
    _MRPin = 1;
}
