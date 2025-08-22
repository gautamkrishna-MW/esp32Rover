#include <stddef.h>
#include <utility>
#include <stdint.h>
#include "motorCoreClass.h"
#include "motorEncoderClass.h"
#include "utilities.h"

class motorControlClass {

private:
  motorCoreClass motor;
  motorEncoderClass encoder;

  float fullRotationPulseCount = 0;
  bool forwardPinState[2] = {0,0};
  bool forwardStateEnabled = false;
  long int pulseCounter = 0;
  
  // Lambda for interrupt callback
  void interruptHandler() {
    uint8_t pinBState = encoder.getPinBState();
    Serial.printf("Pin State: %d\n", pulseCounter);
    if (pinBState > 0) {
      pulseCounter++;
    }
    else {
      pulseCounter--;
    }
  }

public:
  // Constructor
  motorControlClass(int pin1, int pin2, int encoderPinA, int encoderPinB) : motor(pin1, pin2), encoder(encoderPinA, encoderPinB) {
    forwardStateEnabled = false;
    fullRotationPulseCount = 0;
    forwardPinState[0] = 0;
    forwardPinState[1] = 0;

    encoder.setCustomISRHandler(isrHandler, this);
    pulseCounter = 0;
  }

  void setPWM(int pwmPin, int channel, int freq, int nBits, int timerNum) {
    motor.setupPWM(pwmPin, channel, freq, nBits, timerNum);
  }

  void setForwardPinState(int pin1State, int pin2State, bool testRun = false) {
    forwardPinState[0] = pin1State;
    forwardPinState[1] = pin2State;
    forwardStateEnabled = true;

    if (testRun) {
      runMotor(forwardPinState[0], forwardPinState[1]);
      delay(50);
      stopMotor();
    }
  }

  void setFullRotationPulseCount_25ga370() {
    fullRotationPulseCount = PULSE_COUNT_GA25_370;
  }

  void setFullRotationPulseCount(float pulseCount){
    fullRotationPulseCount = pulseCount;
  }

  static void isrHandler(void* arg) {
    motorControlClass* thisPtr = static_cast<motorControlClass*>(arg);
    thisPtr->interruptHandler();
  }

  void moveWithPulseCount(int count, int speed = 50) {
    encoder.setupInterrupt();
    int startCount = pulseCounter;
    while (true) {
      if (count > 0) {
        runMotor(forwardPinState[0], forwardPinState[1], speed);
        DEBUG("Pulse counting forward");
      }
      else {
        runMotor(!forwardPinState[0], !forwardPinState[1], speed);
        DEBUG("Pulse counting backward");
      }
      Serial.printf("Pulse count: %d\n", pulseCounter - startCount);
      if (abs(pulseCounter - startCount) >= count)
          break;
    }
    encoder.removeInterrupt();
    stopMotor();
  }

  void moveNumRotations(float numRotations) {
    if (fullRotationPulseCount > 0) {
      int pulseCount = fullRotationPulseCount*numRotations;
      moveWithPulseCount(pulseCount);
    }
    else
      DEBUG("Set full rotation pulse count");
  }

  void runMotor(int pin1State, int pin2State, int speed = 50) {
    motor.setSpeed(speed);
    motor.setMotorPinValues(pin1State, pin2State);
    Serial.printf("P1: %d, P2 %d, Speed: %d\n", pin1State, pin2State, speed);
  }

  void stopMotor() {
    motor.setMotorPinValues(0, 0);
  }
};