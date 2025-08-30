
#pragma once

#include <memory>
#include <string>
#include <queue>

extern "C" {
	#include "esp_log.h"
	#include "driver/gpio.h"
	#include "driver/ledc.h"
	#include "freertos/FreeRTOS.h"
	#include "freertos/task.h"

	static const char* TAG = "ESPRover";
	extern inline void espAssert(esp_err_t code, const char *file, int line, bool abort=true) {
		if (code != ESP_OK) 
		{
			ESP_LOGE(TAG, "ESPAssert: %s %s %d\n", esp_err_to_name(code), file, line);
			if (abort) 
				exit(code);
		}
	}
	#define espErrChk(ans) { espAssert((ans), __FILE__, __LINE__); }
}


// Motor Pins
#define LEFT_MOTOR_PIN1 GPIO_NUM_18 
#define LEFT_MOTOR_PIN2 GPIO_NUM_19
#define RIGHT_MOTOR_PIN1 GPIO_NUM_16
#define RIGHT_MOTOR_PIN2 GPIO_NUM_17

// Motor Forward direction flag configuration
#define DIRECTION_FRONT_LEFT 0 
#define DIRECTION_FRONT_RIGHT 0 
#define DIRECTION_REAR_LEFT 0 
#define DIRECTION_REAR_RIGHT 0 

// Encoder Pins
#define FRONT_LEFT_ENCODER_A GPIO_NUM_26
#define FRONT_LEFT_ENCODER_B GPIO_NUM_27 
#define FRONT_RIGHT_ENCODER_A GPIO_NUM_23 
#define FRONT_RIGHT_ENCODER_B GPIO_NUM_32
#define REAR_LEFT_ENCODER_A GPIO_NUM_14 
#define REAR_LEFT_ENCODER_B GPIO_NUM_13 
#define REAR_RIGHT_ENCODER_A GPIO_NUM_25 
#define REAR_RIGHT_ENCODER_B GPIO_NUM_33 

// PWM Pins
#define PWM_LEFT_PIN GPIO_NUM_2
#define PWM_RIGHT_PIN GPIO_NUM_4
#define PWM_FREQ 5000
#define PWM_BIT_RESOLUTION 8
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define PWM_CLOCK_CONFIG LEDC_USE_APB_CLK
#define PWM_CLOCK_FREQ LEDC_APB_CLK
#define PWM_LEFT_CHANNEL LEDC_CHANNEL_0
#define PWM_RIGHT_CHANNEL LEDC_CHANNEL_1
#define PWM_LEFT_TIMER LEDC_TIMER_0
#define PWM_RIGHT_TIMER LEDC_TIMER_1

// Encoder pulse count
#define FULL_ROTATION_PULSE_COUNT PULSE_COUNT_GA25_370
#define PULSE_COUNT_GA25_370 370

// I2C Comms
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// UART Comms
#define UART_BAUD_RATE 115200
#define UART_BUFF_SIZE (1024 * 2)
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3

// Orintation Sensor
#define SENSOR_9AXIS_ADDRESS 0x28