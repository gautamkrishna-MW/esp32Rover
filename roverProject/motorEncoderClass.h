#include <stdint.h>
#include "Arduino.h"
#include "esp32-hal-gpio.h"

#include <stdlib.h>

// Clas for encoder sensors
class motorEncoderClass {

private:
  int encoderPinA = 0;
  int encoderPinB = 0;
  void (*interruptFcn)(void*) = nullptr;
  void* interruptArg = nullptr;
  bool directionAB = 0;
  
public:
  motorEncoderClass(int pinA, int pinB) {
    encoderPinA = pinA;
    encoderPinB = pinB;

    interruptFcn = defaultHandler;
    interruptArg = this;

    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    // Check if the pin assigned handles hardware interrupts
    if (digitalPinToInterrupt(encoderPinA) < 0) {
      Serial.println("Failed to recognize pin as interrupt");
    }
    else
      setupInterrupt();
  }

  void setupInterrupt() {
    attachInterruptArg(encoderPinA, interruptFcn, interruptArg, RISING);
  }

  void removeInterrupt() {
    detachInterrupt(encoderPinA);
  }

  int getPinBState() {
    return digitalRead(encoderPinB);
  }

  static void defaultHandler(void* arg) {
    motorEncoderClass* objPtr = static_cast<motorEncoderClass*>(arg);
    objPtr->defaultISRHandler();
  }

  void defaultISRHandler() {
    uint8_t pinBState = getPinBState();
    if (pinBState > 0) {
      directionAB = false;
    }
    else {
      directionAB = true;
    }
  }

  void setCustomISRHandler(void(*customHandlerFcn)(void*), void* args) {
    interruptFcn = customHandlerFcn;
    interruptArg = args;
  }
};