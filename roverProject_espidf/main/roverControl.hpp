
#pragma once

#include "utilities.hpp"
#include "roverClass.hpp"
#include "commandQueue.hpp"

class roverControl {
    
    // Rover
    std::unique_ptr<roverClass> baseRover;
    std::unique_ptr<RoverCommandQueue> cmdQueue;

    // UART Serial comms
    std::shared_ptr<uartCommsClass> uartComms;
    uint8_t uartBuffer[UART_BUFF_SIZE]{};
    std::queue<std::string> messageQueue;

public:
    roverControl() {
        ESP_LOGI(TAG, "ESP32 Rover!!!\n");
        baseRover = std::make_unique<roverClass>();
        cmdQueue = std::make_unique<RoverCommandQueue>();
        uartComms = std::make_shared<uartCommsClass>();
        messageQueue = std::queue<std::string>();
        ESP_LOGI(TAG, "Rover setup complete!!!\n");
    }

    uint8_t* getBufferPointer() {
        return &uartBuffer[0];
    }

    void createCommandQueueFromSerial() {
        // Read UART
        uint8_t ch = 0;
        uint32_t stringLen = 0;
        while(uartComms->uart_read(&ch, 1) > 0) {
            uartBuffer[stringLen] = ch;
            if (ch == 'E') {
                break;
            }
            stringLen++;
        }

        // Convert serial data to string and create command queue
        std::string inputStr((const char*)&uartBuffer[0], stringLen);
        std::stringstream ss(inputStr);
        std::string token;
        while(std::getline(ss, token, '\n')) {
            cmdQueue->parseAndQueueCommandString(token.c_str());
        }
    }

    void executeCommands() {
        if (!cmdQueue->hasCommand()) {
            ESP_LOGI(TAG, "Waiting for command\n");
            return;
        }

        RoverCommand cmdStruct = cmdQueue->getNextCommand();
        switch(cmdStruct.cmd) {
            case Command::FORWARD:
                baseRover->forward(cmdStruct.pulseCount, cmdStruct.speed);
                break;
            case Command::BACKWARD:
                baseRover->reverse(cmdStruct.pulseCount, cmdStruct.speed);
                break;
            case Command::LEFT:
                baseRover->left(cmdStruct.pulseCount, cmdStruct.speed);
                break;
            case Command::RIGHT:
                baseRover->right(cmdStruct.pulseCount, cmdStruct.speed);
                break;
            case Command::STOP:
                baseRover->stop();
                break;
            case Command::INVALID:
                break;
        }
    }

    void createSensorDataQueue() {        
        char tempBuff[UART_BUFF_SIZE]{};
        float w{}, x{}, y{}, z{};

        // Euler angles
        baseRover->getEulerOrientation(x, y, z);
        snprintf(tempBuff, sizeof(tempBuff), "Euler: [%f, %f, %f]\n", x, y, z);
        messageQueue.emplace(tempBuff);

        // Quaternion angles
        baseRover->getQuatAngles(w,x,y,z);
        snprintf(tempBuff, sizeof(tempBuff), "Quat: [%f, %f, %f, %f]\n", w, x, y, z);
        messageQueue.emplace(tempBuff);
    }

    void transmitMessageQueue() {
        while (!messageQueue.empty()) {
            uartComms->uart_print(messageQueue.front().c_str());
            messageQueue.pop();
        }
    }

    void run() {
        // createCommandQueueFromSerial();
        // executeCommands();
        createSensorDataQueue();
        transmitMessageQueue();
    }
};