#include "soc/soc.h"
#include "driver/ledc.h"
#include "Arduino.h"
#include "utilities.h"

#define USE_ESP_IDF_LOG

class motorCoreClass {

private:
  int motorPin1 = 0;
  int motorPin2 = 0;
  int motorSpeedVal = 0;
  int fullSpeedDutyCycle = 0;

  bool pwmEnabled = false;
  ledc_timer_config_t pwmTimerConfig;
  ledc_channel_config_t pwmChannelConfig;

public:
  
  // Constructor
  motorCoreClass(int pin1, int pin2): motorPin1(0), motorPin2(0), motorSpeedVal(0), fullSpeedDutyCycle(0) {
    motorPin1 = pin1;
    motorPin2 = pin2;

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pwmEnabled = false;
  }

  void setupPWM(int pwmPinInp, int channel, int freq, int nBits, int timerNum) {
    
    // Setup PWM timer
    pwmTimerConfig.speed_mode = PWM_SPEED_MODE;
    pwmTimerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(nBits);
    pwmTimerConfig.freq_hz = freq;
    pwmTimerConfig.timer_num = static_cast<ledc_timer_t>(timerNum);
    pwmTimerConfig.clk_cfg = PWM_CLOCK_CONFIG;
    pwmTimerConfig.deconfigure = false;

    if (ledc_timer_config(&pwmTimerConfig) != ESP_OK) {
      nBits = ledc_find_suitable_duty_resolution(PWM_CLOCK_FREQ, pwmTimerConfig.freq_hz);
    }
    pwmTimerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(nBits);
    assert(ledc_timer_config(&pwmTimerConfig) == ESP_OK);
    esp_err_t retT = ledc_timer_config(&pwmTimerConfig);
    if (retT != ESP_OK) {
        ESP_LOGE("LEDC", "Failed to configure timer: %s", esp_err_to_name(retT));
        return;
    }

    // Setup PWM Channel
    pwmChannelConfig.channel = static_cast<ledc_channel_t>(channel);
    pwmChannelConfig.gpio_num = pwmPinInp;
    pwmChannelConfig.speed_mode = PWM_SPEED_MODE;
    pwmChannelConfig.timer_sel = static_cast<ledc_timer_t>(timerNum);
    pwmChannelConfig.duty = 0;
    pwmChannelConfig.intr_type = LEDC_INTR_DISABLE;
    pwmChannelConfig.hpoint = 0;
    pwmChannelConfig.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    
    retT = ledc_channel_config(&pwmChannelConfig);
    if (retT != ESP_OK) {
        ESP_LOGE("LEDC", "Failed to configure channel: %s", esp_err_to_name(retT));
        return;
    }

    fullSpeedDutyCycle = (1 << nBits) - 1;
    pwmEnabled = true;
  }

  // Set motor Speed
  void setSpeed(int speedVal) {
    motorSpeedVal = std::min(std::max(0,speedVal),100);
    int speedDutyCycle = (1 - ((float)motorSpeedVal/100.f)) * fullSpeedDutyCycle;
    esp_err_t retT = ledc_set_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel, speedDutyCycle);
    if (retT != ESP_OK) {
        ESP_LOGE("LEDC", "Failed to set duty cycle: %s", esp_err_to_name(retT));
        return;
    }

    retT = ledc_update_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel);
    if (retT != ESP_OK) {
        ESP_LOGE("LEDC", "Failed to update duty cycle: %s", esp_err_to_name(retT));
        return;
    }

    Serial.printf("Speed Set at: %d\n", speedDutyCycle);
  }

  // Get motor Speed
  int getSpeed() {
    return motorSpeedVal;
  }

  // Set motor pin values
  void setMotorPinValues(int pin1State, int pin2State) {
    digitalWrite(motorPin1, pin1State);
    digitalWrite(motorPin2, pin2State);
  }
};

