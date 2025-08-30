
#pragma once

#include "utilities.hpp"

extern "C" void IRAM_ATTR defaultHandler(void* arg);

// Class for encoder sensors
class motorEncoderClass {

private:
	gpio_num_t encoderPinA;
	gpio_num_t encoderPinB;
	void (*interruptFcn)(void*) = nullptr;
	void* interruptArg = nullptr;
	
public:
	// Encoder specific variables
	volatile long pulseCount = 0;

	motorEncoderClass(gpio_num_t pinA, gpio_num_t pinB) : encoderPinA(pinA), encoderPinB(pinB) {

		interruptFcn = defaultHandler;
		interruptArg = this;

		// Enable interrupts on encoder pins
		espErrChk(gpio_set_direction(encoderPinA, GPIO_MODE_INPUT));
		espErrChk(gpio_set_direction(encoderPinB, GPIO_MODE_INPUT));
		espErrChk(gpio_set_intr_type(encoderPinA, GPIO_INTR_POSEDGE));
		espErrChk(gpio_isr_handler_add(pinA, interruptFcn, interruptArg));
				
		setupInterrupt();
		ESP_LOGI(TAG, "Motor encoder setup successful\n");
	}

	void setupInterrupt() {
		espErrChk(gpio_intr_enable(encoderPinA));
	}

	void removeInterrupt() {
		espErrChk(gpio_intr_disable(encoderPinA));
	}

	const long int getPulseCount() {
		return pulseCount;
	}

	gpio_num_t getEncoderBPin() {
		return encoderPinB;
	}

	void setCustomISRHandler(void(*customHandlerFcn)(void*), void* args) {
		interruptFcn = customHandlerFcn;
		interruptArg = args;
	}
};

extern "C" {
	void IRAM_ATTR defaultHandler(void* arg) {
		motorEncoderClass* self = static_cast<motorEncoderClass*>(arg);
		int pinBState = gpio_get_level(self->getEncoderBPin());
		if (pinBState) {
			self->pulseCount++;
		}
		else {
			self->pulseCount--;
		}
	}
}