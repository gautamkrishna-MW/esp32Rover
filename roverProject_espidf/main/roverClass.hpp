
#pragma once

#include <memory>
#include <iostream>

#include "utilities.hpp"
#include "motorControlClass.hpp"
#include "bno055SensorClass.hpp"
#include "uartCommsClass.hpp"

class roverClass {
private:
  // Motor setup
  std::unique_ptr<motorControlClass> frontRight;
  std::unique_ptr<motorControlClass> frontLeft;
  std::unique_ptr<motorControlClass> rearRight;
  std::unique_ptr<motorControlClass> rearLeft;

  // I2C orientation sensor setup
  std::shared_ptr<bno055SensorClass> sensor;

public:
  float orientation[4];
  
  roverClass() {
    ESP_LOGI(TAG, "Setting up rover...\n");
    espErrChk(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    
    frontLeft = std::make_unique<motorControlClass>(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, FRONT_LEFT_ENCODER_A, FRONT_LEFT_ENCODER_B);
    frontLeft->setPWM(PWM_LEFT_PIN, PWM_LEFT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_LEFT_TIMER);
    frontLeft->setFullRotationPulseCount();
    frontLeft->setForwardPinState(DIRECTION_FRONT_LEFT, !DIRECTION_FRONT_LEFT,true);
    
    frontRight = std::make_unique<motorControlClass>(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, FRONT_RIGHT_ENCODER_A, FRONT_RIGHT_ENCODER_B);
    frontRight->setPWM(PWM_RIGHT_PIN, PWM_RIGHT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_RIGHT_TIMER);
    frontRight->setFullRotationPulseCount();
    frontRight->setForwardPinState(DIRECTION_FRONT_RIGHT, !DIRECTION_FRONT_RIGHT,true);
    
    rearLeft = std::make_unique<motorControlClass>(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, REAR_LEFT_ENCODER_A, REAR_LEFT_ENCODER_B);
    rearLeft->setPWM(PWM_LEFT_PIN, PWM_LEFT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_LEFT_TIMER);
    rearLeft->setFullRotationPulseCount();
    rearLeft->setForwardPinState(DIRECTION_REAR_LEFT, !DIRECTION_REAR_LEFT,true);
    
    rearRight = std::make_unique<motorControlClass>(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, REAR_RIGHT_ENCODER_A, REAR_RIGHT_ENCODER_B);
    rearRight->setPWM(PWM_RIGHT_PIN, PWM_RIGHT_CHANNEL, PWM_FREQ, PWM_BIT_RESOLUTION, PWM_RIGHT_TIMER);
    rearRight->setFullRotationPulseCount();
    rearRight->setForwardPinState(DIRECTION_REAR_RIGHT, !DIRECTION_REAR_RIGHT,true);
    
    ESP_LOGI(TAG, "Motor Setup successful\n");
    ESP_LOGI(TAG, "PWM Setup successful\n");
    ESP_LOGI(TAG, "Full rotation pulse count set to %d\n", (int)FULL_ROTATION_PULSE_COUNT);
    ESP_LOGI(TAG, "Motor direction configuration done\n");

    sensor = std::make_shared<bno055SensorClass>(SENSOR_9AXIS_ADDRESS);
    orientation[0] = 0; orientation[1] = 0; orientation[2] = 0; orientation[3] = 0;
    ESP_LOGI(TAG, "Orientation sensor setup successful\n");

    ESP_LOGI(TAG, "Rover setup successful\n");
  }

  void getEulerOrientation(float& yaw, float& roll, float& pitch) {
    sensor->getEulerAngles(yaw, roll, pitch);
  }

  void getQuatAngles(float& w, float& x, float& y, float& z) {
    sensor->getQuatAngles(w, x, y, z);
  }

  bool isSensorCalibrated() {
    return sensor->isSensorCalibrated();
  }

  void forward(int count, int speed) {
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(count, speed);
  }

  void reverse(int count, int speed) {
    if (count > 0)
      count *= -1;
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(count, speed);
  }

  void left(int count, int speed) {
    frontLeft->moveWithPulseCount(-count, speed);
    frontRight->moveWithPulseCount(count, speed);
    rearLeft->moveWithPulseCount(-count, speed);
    rearRight->moveWithPulseCount(count, speed);
  }

  void right(int count, int speed) {
    frontLeft->moveWithPulseCount(count, speed);
    frontRight->moveWithPulseCount(-count, speed);
    rearLeft->moveWithPulseCount(count, speed);
    rearRight->moveWithPulseCount(-count, speed);
  }

  void stop() {
    frontLeft->stopMotor();
    frontRight->stopMotor();
    rearLeft->stopMotor();
    rearRight->stopMotor();
  }

  ~roverClass() {}
};