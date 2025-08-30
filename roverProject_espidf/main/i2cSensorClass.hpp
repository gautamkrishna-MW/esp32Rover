
#pragma once

#include <stdint.h>
#include "i2cCommsClass.hpp"

// Abstract base class for I2C sensors on ESP32
class i2cSensorClass {

private:
  i2cCommClass commObject;

  uint8_t* commDataRegPtr = nullptr;
  uint8_t sensorAddress = 0;
  
public:
  i2cSensorClass(uint8_t address): commObject(address) {}

  // Get buffer pointer
  uint8_t* getDataPtr() {
    return commObject.getBufferPtr();
  }

  // Name-value pair write to sensor
  void writeRegister(uint8_t regAddress, uint8_t regValue, int delayVal = 5) {
    commObject.writeRegister(regAddress, regValue, delayVal);
  }

  // Read register data
  uint8_t* readRegister(uint8_t regAddress, uint8_t len = 1, int delayVal = 5) {
    commObject.readData(regAddress, len, delayVal);
    return commDataRegPtr;
  }
};