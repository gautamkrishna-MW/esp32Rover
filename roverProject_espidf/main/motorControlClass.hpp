#pragma once

#include "utilities.hpp"
#include "motorCoreClass.hpp"
#include "motorEncoderClass.hpp"

class motorControlClass {

private:
	
	std::unique_ptr<motorCoreClass> motor;
	std::unique_ptr<motorEncoderClass> encoder;

	float fullRotationPulseCount = 0;
	bool forwardPinState[2] = {0,0};
	bool forwardStateEnabled = false;
	
public:
	// Constructor
	motorControlClass(gpio_num_t pin1, gpio_num_t pin2, gpio_num_t encoderPinA, gpio_num_t encoderPinB) {

		motor = std::make_unique<motorCoreClass>(pin1, pin2);
		encoder = std::make_unique<motorEncoderClass>(encoderPinA, encoderPinB);

		forwardStateEnabled = false;
		fullRotationPulseCount = 0;
		forwardPinState[0] = 0;
		forwardPinState[1] = 0;
		encoder->setupInterrupt();
		
		ESP_LOGI(TAG, "Motor control setup successful\n");
	}

	~motorControlClass(){
		encoder->removeInterrupt();
	}

	void setPWM(int pwmPin, int channel, int freq, int nBits, int timerNum) {
		motor->setupPWM(pwmPin, channel, freq, nBits, timerNum);
	}

	void setForwardPinState(int pin1State, int pin2State, bool testRun = false) {
		forwardPinState[0] = pin1State;
		forwardPinState[1] = pin2State;
		forwardStateEnabled = true;

		if (testRun) {
			runMotor(forwardPinState[0], forwardPinState[1]);
			vTaskDelay(pdMS_TO_TICKS(50));
			stopMotor();
		}
	}

	void setFullRotationPulseCount(int pulseCount = FULL_ROTATION_PULSE_COUNT) {
		fullRotationPulseCount = pulseCount;
	}

	void moveWithPulseCount(long int count, int speed = 50) {
		long int startCount = encoder->getPulseCount();
		long int target = startCount + count;

		if (count > 0) {
			runMotor(forwardPinState[0], forwardPinState[1], speed);
			while (encoder->getPulseCount() < target) {
				vTaskDelay(1);
			}
		} else {
			runMotor(!forwardPinState[0], !forwardPinState[1], speed);
			while (encoder->getPulseCount() > target) {
				vTaskDelay(1);
			}
		}
		stopMotor();
	}

	void moveNumRotations(float numRotations) {
		if (fullRotationPulseCount > 0) {
			int pulseCount = fullRotationPulseCount*numRotations;
			moveWithPulseCount(pulseCount);
		}
	}

	void runMotor(int pin1State, int pin2State, int speed = 50) {
		motor->setSpeed(speed);
		motor->setMotorPinValues(pin1State, pin2State);
	}

	void stopMotor() {
		motor->setMotorPinValues(0, 0);
	}
};