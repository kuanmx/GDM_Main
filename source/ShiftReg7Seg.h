//
// Created by poh14 on 3/29/2018.
//

#ifndef SHIFTREG7SEG_H
#define SHIFTREG7SEG_H

#include <mbed.h>
#include <vector>
#include <memory>

namespace{
    //bytes are in order of: g, f, e, d, c, b, a, DP
    const uint8_t characterMap[] = {0b01111110, //0
                                    0b00001100, //1
                                    0b10110110, //2
                                    0b10011110, //3
                                    0b11001100, //4
                                    0b11011010, //5
                                    0b11111010, //6
                                    0b00001110, //7
                                    0b11111110, //8
                                    0b11011110, //9
                                    0b11110010, //Error
    };
}

class ShiftReg7Seg {
public:
    ShiftReg7Seg() = delete;
//    ShiftReg7Seg(SPI *spiObj, PinName latchPin, unsigned int numberOfDisplay = 4, PinName MRPin = NC);
    ShiftReg7Seg(PinName MOSI_Pin, PinName MISO_Pin, PinName SCK_Pin, PinName latchPin, unsigned int numberOfDisplay = 4, PinName MRPin = NC);

    //public methods
    std::vector<uint8_t> display(double value);
    void clearAll();

    //getters & setters
    unsigned int getNumberOfDisplay() const;
    void setNumberOfDisplay(unsigned int _numberOfDisplay);
    int getNumberOfDP() const;
    void setNumberOfDP(int _numberOfDP);

private:
    std::vector<uint8_t> displayDigits(std::vector<uint8_t>& digitArray);
    std::unique_ptr<SPI> _spiPtr;
    DigitalOut _latchPin;
    DigitalOut _MRPin;
    unsigned int _numberOfDisplay = 4;
    int _numberOfDP = 0;


};

#endif //SHIFTREG7SEG_H
