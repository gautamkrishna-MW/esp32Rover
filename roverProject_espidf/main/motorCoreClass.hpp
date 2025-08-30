#pragma once

#include "utilities.hpp"

class motorCoreClass {

private:
  gpio_num_t motorPin1;
  gpio_num_t motorPin2;
  int motorSpeedVal = 0;
  int fullSpeedDutyCycle = 0;

  bool pwmEnabled = false;
  ledc_timer_config_t pwmTimerConfig;
  ledc_channel_config_t pwmChannelConfig;

public:
  
  // Constructor
  motorCoreClass(gpio_num_t pin1, gpio_num_t pin2): motorPin1(GPIO_NUM_NC), motorPin2(GPIO_NUM_NC), motorSpeedVal(0), fullSpeedDutyCycle(0) {
    motorPin1 = pin1;
    motorPin2 = pin2;

    espErrChk(gpio_set_direction(motorPin1, GPIO_MODE_OUTPUT));
    espErrChk(gpio_set_direction(motorPin2, GPIO_MODE_OUTPUT));
    pwmEnabled = false;
    ESP_LOGI(TAG, "Motor Core Initialized\n");
  }

  void setupPWM(int pwmPinInp, int channel, int freq, int nBits, int timerNum) {    
    // Setup PWM timer
    pwmTimerConfig.speed_mode = PWM_SPEED_MODE;
    pwmTimerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(nBits);
    pwmTimerConfig.freq_hz = freq;
    pwmTimerConfig.timer_num = static_cast<ledc_timer_t>(timerNum);
    pwmTimerConfig.clk_cfg = PWM_CLOCK_CONFIG;
    pwmTimerConfig.deconfigure = false;

	// Check if bits set are suitable for the clock frequency
    if (ledc_timer_config(&pwmTimerConfig) != ESP_OK) {
		nBits = ledc_find_suitable_duty_resolution(PWM_CLOCK_FREQ, pwmTimerConfig.freq_hz);
		ESP_LOGI(TAG, "Updating PWM bit resolution to %d", nBits);
		pwmTimerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(nBits);
	}
    espErrChk(ledc_timer_config(&pwmTimerConfig));
    
    // Setup PWM Channel
    pwmChannelConfig.channel = static_cast<ledc_channel_t>(channel);
    pwmChannelConfig.gpio_num = pwmPinInp;
    pwmChannelConfig.speed_mode = PWM_SPEED_MODE;
    pwmChannelConfig.timer_sel = static_cast<ledc_timer_t>(timerNum);
    pwmChannelConfig.duty = 0;
    pwmChannelConfig.intr_type = LEDC_INTR_DISABLE;
    pwmChannelConfig.hpoint = 0;
    pwmChannelConfig.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    espErrChk(ledc_channel_config(&pwmChannelConfig));
    
    fullSpeedDutyCycle = (1 << nBits) - 1;
    pwmEnabled = true;
  }

  // Set motor Speed
  void setSpeed(int speedVal) {
    // Return if PWM not enabled
    if (!pwmEnabled) {
      ESP_LOGI(TAG, "PWM not enabled");
      return;
    }

    motorSpeedVal = std::min(std::max(0,speedVal),100);
    int speedDutyCycle = (1 - ((float)motorSpeedVal/100.f)) * fullSpeedDutyCycle;
    espErrChk(ledc_set_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel, speedDutyCycle));
    espErrChk(ledc_update_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel));
  }

  // Get motor Speed
  int getSpeed() {
    return motorSpeedVal;
  }

  // Set motor pin values
  void setMotorPinValues(int pin1State, int pin2State) {
    gpio_set_level(motorPin1, pin1State);
    gpio_set_level(motorPin2, pin2State);
  }
};