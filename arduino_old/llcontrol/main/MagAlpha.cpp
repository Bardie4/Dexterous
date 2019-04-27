/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 3rd generation Sensors. MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
****************************************************/


#include "MagAlpha.h"

//MagAlpha Read/Write Register Command
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define SSI_MODE            SPI_MODE1

MagAlpha::MagAlpha(){
}

void MagAlpha::begin(uint8_t spiChipSelectPin, SPIClass * spi){
    setSpiChipSelectPin(spiChipSelectPin);
    _spi = spi;
    _speedMaximum = 10000000;
    //_spiMode = MA_SPI_MODE_3;
    _spi->begin();
    //_spi->beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
}

// void  MagAlpha::begin(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin){
//     setSpiChipSelectPin(spiChipSelectPin);
//     _speedMaximum = spiSclkFrequency;
//     _spiMode = spiMode;
//     spi->begin();
//     spi->beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
// }

void MagAlpha::end(){
    _spi->end();
}

double MagAlpha::readAngle(){
  uint16_t angle;
  double angleInDegree;
  angle = readAngleRaw16();
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}

uint16_t MagAlpha::readAngleRaw(){
    return readAngleRaw16();
}

uint16_t MagAlpha::readAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlpha::readAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer(0x00);     //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t MagAlpha::readAngleRaw(bool* error){
    uint16_t angle;
    uint8_t parity;
    uint8_t highStateCount = 0;

    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000);
    parity = _spi->transfer(0x00);
    digitalWrite(_spiChipSelectPin, HIGH);

    parity = ((parity & 0x80) >> 7);
    //Count the number of 1 in the angle binary value
    for (int i=0;i<16;++i){
        if ((angle & (1 << i)) != 0){
            highStateCount++;
        }
    }
    //check if parity bit is correct
    if ((highStateCount % 2) == 0){
        if (parity == 0){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    else{
        if (parity == 1){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    return angle;
}

uint8_t MagAlpha::readRegister(uint8_t address){
  uint8_t readbackRegisterValue;
  digitalWrite(_spiChipSelectPin, LOW);
  _spi->transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
  digitalWrite(_spiChipSelectPin, HIGH);
  digitalWrite(_spiChipSelectPin, LOW);
  readbackRegisterValue = ((_spi->transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(_spiChipSelectPin, HIGH);
  return readbackRegisterValue;
}

uint8_t MagAlpha::writeRegister(uint8_t address, uint8_t value){
  uint8_t readbackRegisterValue;
  digitalWrite(_spiChipSelectPin, LOW);
  _spi->transfer16(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
  digitalWrite(_spiChipSelectPin, HIGH);
  delay(20);                      //Wait for 20ms
  digitalWrite(_spiChipSelectPin, LOW);
  readbackRegisterValue = ((_spi->transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(_spiChipSelectPin, HIGH);
  //readbackRegisterValue should be equal to the written value
  return readbackRegisterValue;
}

void MagAlpha::setSpiClockFrequency(uint32_t speedMaximum){
    _speedMaximum = speedMaximum;
    //_spi->beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
}

void MagAlpha::setSpiDataMode(uint8_t spiMode){
    _spiMode = spiMode;
    _spi->setDataMode(_spiMode);
}

void MagAlpha::setSpiChipSelectPin(uint8_t spiChipSelectPin){
    _spiChipSelectPin = spiChipSelectPin;
    pinMode(_spiChipSelectPin, OUTPUT);
    digitalWrite(_spiChipSelectPin, HIGH);
}

double MagAlpha::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
    double angleInDegree;
    angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
    return angleInDegree;
}