

#pragma once

#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <sstream>
#include <regex>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Plugin.h"
#include "motorClass.h"

#define MESSAGE_BUFFER_LEN 100

extern "C" {

    using gpioVect = std::vector<gpio_num_t>;
    using vect8 = std::vector<uint8_t>;

    void readMessageIntoQueue(void* inpArgs) {
        MotorPlugin* motor_ptr = static_cast<MotorPlugin*>(inpArgs);

        while(true) {
            if (!motor_ptr->get_msgBuffer_ptr()->empty()) {
                for (auto& msg : *(motor_ptr->get_msgBuffer_ptr())) {
                    motor_ptr->parseMessage(msg);
                }
            }
        }
    }

    class MotorPlugin : public Plugin {
        
        // Motor objects
        std::unique_ptr<Motor> frontLeft;
        std::unique_ptr<Motor> frontRight;
        std::unique_ptr<Motor> rearLeft;
        std::unique_ptr<Motor> rearRight;

        // Message stack related variables
        char delimiter;
        TaskHandle_t cmdQ_task_handle;
        
        // Rover commands
        struct command {char dir; int pulseCount; int speed;};
        QueueHandle_t msgQ_handle;
        
    public:
        MotorPlugin(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> log, char cmd_delimiter = '\n') : Plugin(name, comms, log), delimiter(cmd_delimiter) {}

        void setupMotors(gpioVect mPins, gpioVect enPins, gpioVect pwmPins, int32_t pulseLowLimit, int32_t pulseHighLimit, uint32_t pwm_freq, ledc_timer_bit_t pwm_bitwidth, ledc_channel_t pwm_channel) {

            frontLeft = std::make_unique<Motor>(logger, mPins[0], mPins[1], enPins[0], enPins[1], pulseLowLimit, pulseHighLimit, pwmPins[0], pwm_freq, pwm_bitwidth, pwm_channel);

            frontRight= std::make_unique<Motor>(logger, mPins[2], mPins[3], enPins[2], enPins[3], pulseLowLimit, pulseHighLimit, pwmPins[1], pwm_freq, pwm_bitwidth, pwm_channel);

            rearLeft = std::make_unique<Motor>(logger, mPins[4], mPins[5], enPins[4], enPins[5], pulseLowLimit, pulseHighLimit, pwmPins[2], pwm_freq, pwm_bitwidth, pwm_channel);

            rearRight = std::make_unique<Motor>(logger, mPins[6], mPins[7], enPins[6], enPins[7], pulseLowLimit, pulseHighLimit, pwmPins[3], pwm_freq, pwm_bitwidth, pwm_channel);

            msgQ_handle = xQueueCreate(MESSAGE_BUFFER_LEN, sizeof(command));
            logger->espAssert(msgQ_handle != NULL);

            xTaskCreate(readMessageIntoQueue, "Message queueing task", 2048, this, 5, &cmdQ_task_handle);
            logger->espAssert(cmdQ_task_handle != NULL);
        }

        bool isValidToken(std::string& token) {
            std::regex pattern("[A-Za-z] \\d{2} \\d{2}\n");
            return std::regex_match(token, pattern);
        }

        void parseMessage(Message& msg) {
            // Copy into string
            std::string commandBuffer(msg.payload.begin(), msg.payload.end());

            // Remove leading and trailing whitespaces
            commandBuffer.erase(0, commandBuffer.find_first_not_of(" "));
            commandBuffer.erase(commandBuffer.find_last_not_of(" ")+1);

            // Parse the string to create a command-queue
            std::istringstream iss(commandBuffer);
            std::string token;
            while (std::getline(iss, token, delimiter)) {
                if (isValidToken(token)) {
                    command cmd;
                    int scanned = sscanf(token.c_str(), "%c %d %d", &cmd.dir, &cmd.pulseCount, &cmd.speed);
                    if (xQueueSend(msgQ_handle, cmd, 100/portTICK_PERIOD_MS ) != pdPASS) {
                        logger->log_info("Motor Plugin", "Message queue overflow.\n");
                    }
                }
            }
        }

        void forward(int pulse_count, int speed) {
            frontLeft->runMotorForward(pulse_count, speed);
            frontRight->runMotorForward(pulse_count, speed);
            rearLeft->runMotorForward(pulse_count, speed);
            rearRight->runMotorForward(pulse_count, speed);
        }

        void reverse(int pulse_count, int speed) {
            frontLeft->runMotorBackward(pulse_count, speed);
            frontRight->runMotorBackward(pulse_count, speed);
            rearLeft->runMotorBackward(pulse_count, speed);
            rearRight->runMotorBackward(pulse_count, speed);
        }

        void left(int pulse_count, int speed) {
            frontLeft->runMotorForward(pulse_count, speed);
            frontRight->runMotorBackward(pulse_count, speed);
            rearLeft->runMotorForward(pulse_count, speed);
            rearRight->runMotorBackward(pulse_count, speed);
        }

        void right(int pulse_count, int speed) {
            frontLeft->runMotorBackward(pulse_count, speed);
            frontRight->runMotorForward(pulse_count, speed);
            rearLeft->runMotorBackward(pulse_count, speed);
            rearRight->runMotorForward(pulse_count, speed);
        }

        void stop() {
            frontLeft->stopMotor();
            frontRight->stopMotor();
            rearLeft->stopMotor();
            rearRight->stopMotor();
        }

        void process() override {

            command cmd;
            if (xQueueReceive(msgQ_handle, &cmd, portMAX_DELAY) == pdTRUE) {
                switch (toupper(cmd.dir)) {
                    case 'F': 
                        forward(cmd.pulseCount, cmd.speed); break;
                    case 'B': 
                        reverse(cmd.pulseCount, cmd.speed); break;
                    case 'L': 
                        left(cmd.pulseCount, cmd.speed); break;
                    case 'R': 
                        right(cmd.pulseCount, cmd.speed); break;
                    case 'S': 
                        stop(); break;
                    default:  
                        stop();
                }
            }
        }

    };

}