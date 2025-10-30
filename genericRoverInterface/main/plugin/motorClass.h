

#pragma once

#include <iostream>
#include <memory>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "driver/pulse_cnt.h"

#include "../utility/Logger.h"

#define DEFAULT_INTR_PRIORITY 10
#define PWM_FREQ 5000
#define PWM_BIT_RESOLUTION 8
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define PWM_CLOCK_CONFIG LEDC_USE_APB_CLK
#define PWM_CLOCK_FREQ LEDC_APB_CLK
#define PWM_TIMER LEDC_TIMER_0

extern "C" {

    static bool motorEventCallback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
        Motor* motorPtr = static_cast<Motor*>(user_ctx);
        motorPtr->stopMotor();
        pcnt_unit_remove_watch_point(unit, edata->watch_point_value);
        motorPtr->resetPulseCount();
        pcnt_unit_clear_count(unit);
    }

    class Motor {
        gpio_num_t motorPinA;
        gpio_num_t motorPinB;
        gpio_num_t encoderPinA;
        gpio_num_t encoderPinB;
        std::shared_ptr<Logger> logger;
        bool invertDirection;
        bool invertCounter;

        // PWM
        gpio_num_t pwmPort;
        uint32_t pwmFreq;
        ledc_timer_bit_t pwmBits;
        ledc_channel_t pwmChannel;
        ledc_timer_config_t pwmTimerConfig;
        ledc_channel_config_t pwmChannelConfig;
        
        // Pulse counter
        int32_t pulseCount;
        int32_t pulseCountLowLimit;
        int32_t pulseCountHighLimit;
        int fullSpeedDutyCycle = 0;

        pcnt_unit_config_t pcnt_unit;
        pcnt_unit_handle_t pcnt_unit_handle;
        pcnt_chan_config_t pcnt_chan_a;
        pcnt_chan_config_t pcnt_chan_b;
        pcnt_channel_handle_t pcnt_chan_handle_a;
        pcnt_channel_handle_t pcnt_chan_handle_b;
        pcnt_glitch_filter_config_t filter_config;

        void setupPCNT() {
            // Set PCNT unit
            pcnt_unit_handle = NULL;
            pcnt_unit.high_limit = pulseCountHighLimit;
            pcnt_unit.low_limit = pulseCountLowLimit;
            pcnt_unit.intr_priority = DEFAULT_INTR_PRIORITY;
            logger->espErrChk(pcnt_new_unit(&pcnt_unit, &pcnt_unit_handle));
            logger->espAssert(pcnt_unit_handle != NULL);

            // Set PCNT glitch filter
            filter_config.max_glitch_ns = 1000;
            logger->espErrChk(pcnt_unit_set_glitch_filter(pcnt_unit_handle, &filter_config));
            
            // Set PCNT channel A and counter behaviour
            pcnt_chan_handle_a = NULL;
            pcnt_chan_a.edge_gpio_num = encoderPinA;
            pcnt_chan_a.level_gpio_num = encoderPinB;
            logger->espErrChk(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a, &pcnt_chan_handle_a));
            logger->espAssert(pcnt_chan_handle_a != NULL);

            if (!invertCounter)
                logger->espErrChk(pcnt_channel_set_edge_action(pcnt_chan_handle_a, 
                    PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
            else
                logger->espErrChk(pcnt_channel_set_edge_action(pcnt_chan_handle_a, 
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            logger->espErrChk(pcnt_channel_set_level_action(pcnt_chan_handle_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, 
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
            
            // Set PCNT channel B and counter behaviour
            pcnt_chan_handle_b = NULL;
            pcnt_chan_b.edge_gpio_num = encoderPinB;
            pcnt_chan_b.level_gpio_num = encoderPinA;
            logger->espErrChk(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b, &pcnt_chan_handle_b));
            logger->espAssert(pcnt_chan_handle_b != NULL);
            
            if (!invertCounter)
                logger->espErrChk(pcnt_channel_set_edge_action(pcnt_chan_handle_b, 
                    PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
            else
                logger->espErrChk(pcnt_channel_set_edge_action(pcnt_chan_handle_b, 
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
            logger->espErrChk(pcnt_channel_set_level_action(pcnt_chan_handle_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, 
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

            // Register event callback for counters with watch-point
            pcnt_event_callbacks_t callbck;
            callbck.on_reach = motorEventCallback;
            logger->espErrChk(pcnt_unit_register_event_callbacks(pcnt_unit_handle, &callbck, this));
        }

        void setupPWM() {
            // Setup PWM timer
            pwmTimerConfig.speed_mode = PWM_SPEED_MODE;
            pwmTimerConfig.duty_resolution = pwmBits;
            pwmTimerConfig.freq_hz = pwmFreq;
            pwmTimerConfig.timer_num = PWM_TIMER;
            pwmTimerConfig.clk_cfg = PWM_CLOCK_CONFIG;
            pwmTimerConfig.deconfigure = false;

            // Check if bits set are suitable for the clock frequency
            if (ledc_timer_config(&pwmTimerConfig) != ESP_OK) {
                uint32_t bits = ledc_find_suitable_duty_resolution(PWM_CLOCK_FREQ, pwmTimerConfig.freq_hz);
                logger->log_info("MotorDriver", "Updating PWM bit resolution to %d", bits);
                pwmBits = static_cast<ledc_timer_bit_t>(bits);
                pwmTimerConfig.duty_resolution = pwmBits;
            }

            // Setup timer configuration
            logger->espErrChk(ledc_timer_config(&pwmTimerConfig));

            // Setup PWM Channel
            pwmChannelConfig.channel = pwmChannel;
            pwmChannelConfig.gpio_num = pwmPort;
            pwmChannelConfig.speed_mode = PWM_SPEED_MODE;
            pwmChannelConfig.timer_sel = static_cast<ledc_timer_t>(PWM_TIMER);
            pwmChannelConfig.duty = 0;
            pwmChannelConfig.intr_type = LEDC_INTR_DISABLE;
            pwmChannelConfig.hpoint = 0;
            pwmChannelConfig.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;

            // Setup channel configuration
            logger->espErrChk(ledc_channel_config(&pwmChannelConfig));
            fullSpeedDutyCycle = (1 << pwmBits) - 1;
            logger->log_info("Motor Driver", "Motor driver ready. \n");
        }

    public:
        Motor(std::shared_ptr<Logger> p_logger, gpio_num_t mPinA, gpio_num_t mPinB, gpio_num_t enPinA, gpio_num_t enPinB, int32_t pulse_low_limit, int32_t pulse_high_limit, gpio_num_t pwm_pin, uint32_t pwm_freq, ledc_timer_bit_t pwm_bitwidth, ledc_channel_t pwm_channel ): 
        
        logger(p_logger), motorPinA(mPinA), motorPinB(mPinB), encoderPinA(enPinA), encoderPinB(enPinB), pulseCountLowLimit(pulse_low_limit), pulseCountHighLimit(pulse_high_limit), pwmPort(pwm_pin), pwmFreq(pwm_freq), pwmBits(pwm_bitwidth), pwmChannel(pwm_channel) {

            // Set flags
            invertDirection = false;
            invertCounter = false;

            // Set GPIO directions
            gpio_set_direction(motorPinA, GPIO_MODE_OUTPUT);
            gpio_set_direction(motorPinB, GPIO_MODE_OUTPUT);
            gpio_set_direction(encoderPinA, GPIO_MODE_INPUT);
            gpio_set_direction(encoderPinB, GPIO_MODE_INPUT);
        }

        // Start the PCNT Unit
        void init() {
            setupPWM();
            setupPCNT();
            logger->espErrChk(pcnt_unit_enable(pcnt_unit_handle));
            logger->espErrChk(pcnt_unit_start(pcnt_unit_handle));
        }

        void resetPulseCount() {
            pulseCount = 0;
        }

        void invertMotorDirection() {
            invertDirection = true;
        }

        void invertPulseCounter() {
            invertCounter = true;
        }

        void setSpeed(int speedVal) {
            speedVal = std::min(std::max(0,speedVal),100);
            int speedDutyCycle = (1 - ((float)speedVal/100.f)) * fullSpeedDutyCycle;

            // Set and update duty cycle
            logger->espErrChk(ledc_set_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel, speedDutyCycle));
            logger->espErrChk(ledc_update_duty(pwmChannelConfig.speed_mode, pwmChannelConfig.channel));
        }

        // Running motor
        void runMotorForward(int pulses, int speedVal) {
            pulseCount = pulses;
            logger->espErrChk(pcnt_unit_add_watch_point(pcnt_unit_handle, pulseCount));
            setSpeed(speedVal);

            logger->espErrChk(gpio_set_level(motorPinA, (uint32_t)invertDirection));
            logger->espErrChk(gpio_set_level(motorPinB, (uint32_t)~invertDirection));
        }

        void runMotorBackward(int pulses, int speedVal) {
            pulseCount = pulses;
            logger->espErrChk(pcnt_unit_add_watch_point(pcnt_unit_handle, pulseCount));
            setSpeed(speedVal);

            logger->espErrChk(gpio_set_level(motorPinA, (uint32_t)~invertDirection));
            logger->espErrChk(gpio_set_level(motorPinB, (uint32_t)invertDirection));
        }

        void stopMotor() {
            gpio_set_level(motorPinA, 0);
            gpio_set_level(motorPinB, 0);
        }

        ~Motor() {
            logger->espErrChk(pcnt_unit_stop(pcnt_unit_handle));
            logger->espErrChk(pcnt_unit_clear_count(pcnt_unit_handle));
            logger->espErrChk(pcnt_unit_remove_all_watch_step(pcnt_unit_handle));
            logger->espErrChk(pcnt_del_channel(pcnt_chan_handle_a));
            logger->espErrChk(pcnt_del_channel(pcnt_chan_handle_b));
            logger->espErrChk(pcnt_unit_disable(pcnt_unit_handle));
            logger->espErrChk(pcnt_del_unit(pcnt_unit_handle));
        }
    };
}