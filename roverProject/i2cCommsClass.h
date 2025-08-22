#include <stdint.h>
#include <Wire.h>
#include "utilities.h"

#define MAX_I2C_BUFFER_SIZE 32

class i2cCommClass {

private:
  uint8_t regDataArray[MAX_I2C_BUFFER_SIZE] = {0};
  uint8_t devAddress;

public:
  i2cCommClass( uint8_t address): devAddress(address) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  }

  uint8_t* getBufferPtr() {
    return regDataArray;
  }

  uint8_t getDevAddress() {
    return devAddress;
  }

  void writeRegister(uint8_t regAddress, uint8_t regValue, int delayVal = 10) {
    Wire.beginTransmission(devAddress);
    Wire.write(regAddress);
    Wire.write(regValue);
    Wire.endTransmission();
    delay(delayVal);
  }

  void readData(uint8_t cmdRegister, uint8_t len = 1) {
    Wire.beginTransmission(devAddress);
    Wire.write(cmdRegister);
    Wire.endTransmission();

    Wire.requestFrom(devAddress, len);
    for (int i=0; i<len; i++) {
        regDataArray[i] = Wire.read();
    }    
  }
};