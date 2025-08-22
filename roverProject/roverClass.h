#include <memory>

#include <iostream>
#include "motorControlClass.h"
#include "bno055SensorClass.h"
#include "utilities.h"

class roverClass {
private:
  // Define motor positions for rover

  std::unique_ptr<motorControlClass> frontRight;
  std::unique_ptr<motorControlClass> frontLeft;
  std::unique_ptr<motorControlClass> rearRight;
  std::unique_ptr<motorControlClass> rearLeft;

  std::shared_ptr<bno055SensorClass> sensor;
  
public:
  float orientation[4];
  
public:
  roverClass() {
    Serial.println("Setting up rover...");

    frontLeft = std::make_unique<motorControlClass>(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, FRONT_LEFT_ENCODER_A, FRONT_LEFT_ENCODER_B);
    frontRight = std::make_unique<motorControlClass>(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, FRONT_RIGHT_ENCODER_A, FRONT_RIGHT_ENCODER_B);
    rearLeft = std::make_unique<motorControlClass>(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, REAR_LEFT_ENCODER_A, REAR_LEFT_ENCODER_B);
    rearRight = std::make_unique<motorControlClass>(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, REAR_RIGHT_ENCODER_A, REAR_RIGHT_ENCODER_B);

    frontLeft->setPWM(LEFT_PWM_PIN, PWM_LEFT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_LEFT_TIMER);
    rearLeft->setPWM(LEFT_PWM_PIN, PWM_LEFT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_LEFT_TIMER);

    frontRight->setPWM(RIGHT_PWM_PIN, PWM_RIGHT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_RIGHT_TIMER);
    rearRight->setPWM(RIGHT_PWM_PIN, PWM_RIGHT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_RIGHT_TIMER);

    frontLeft->setFullRotationPulseCount_25ga370();
    rearLeft->setFullRotationPulseCount_25ga370();
    frontRight->setFullRotationPulseCount_25ga370();
    rearRight->setFullRotationPulseCount_25ga370();

    frontLeft->setForwardPinState(1,0,true);
    rearLeft->setForwardPinState(1,0,true);
    frontRight->setForwardPinState(1,0,true);
    rearRight->setForwardPinState(1,0,true);

    sensor = std::make_shared<bno055SensorClass>(SENSOR_9AXIS_ADDRESS);
    orientation[0] = 0; orientation[1] = 0; orientation[2] = 0; orientation[3] = 0;
  }

  void getEulerOrientation() {
    sensor->getEulerAngles(orientation[0], orientation[1], orientation[2]);
  }

  void getQuatOrientation() {
    sensor->getQuatAngles(orientation[0], orientation[1], orientation[2], orientation[3]);
  }

  bool isSensorCalibrated() {
    return sensor->isSensorCalibrated();
  }

  void forward(int count, int speed) {
  #ifdef TEST_MODE
    frontLeft->runMotor(1, 0, speed);
    frontRight->runMotor(1, 0, speed);
    rearLeft->runMotor(1, 0, speed);
    rearRight->runMotor(1, 0, speed);
  #else
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(count, speed);
  #endif
  }

  void reverse(int count, int speed) {
  #ifdef TEST_MODE
    frontLeft->runMotor(0, 1, speed);
    frontRight->runMotor(0, 1, speed);
    rearLeft->runMotor(0, 1, speed);
    rearRight->runMotor(0, 1, speed);
  #else
    if (count > 0)
      count *= -1;
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(count, speed);
  #endif
  }

  void left(int count, int speed) {
  #ifdef TEST_MODE
    frontLeft->runMotor(1, 0, speed);
    frontRight->runMotor(0, 1, speed);
    rearLeft->runMotor(1, 0, speed);
    rearRight->runMotor(0, 1, speed);
  #else
    frontLeft->moveWithPulseCount(-count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(-count, speed);
    rearRight->moveWithPulseCount(count, speed);
  #endif
  }

  void right(int count, int speed) {
  #ifdef TEST_MODE
    frontLeft->runMotor(0, 1, speed);
    frontRight->runMotor(1, 0, speed);
    rearLeft->runMotor(0, 1, speed);
    rearRight->runMotor(1, 0, speed);
  #else
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(-count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(-count, speed);
  #endif
  }

  void stop() {
    frontLeft->stopMotor();
    frontRight->stopMotor();
    rearLeft->stopMotor();
    rearRight->stopMotor();
  }

  ~roverClass() {}
};