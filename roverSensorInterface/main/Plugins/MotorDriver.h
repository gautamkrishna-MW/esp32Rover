
#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <sstream>

#include "PluginClass.h"

#define I2C_MOTOR_SPEED_CONTROL_REG 0x06
#define I2C_MOTOR_PWM_CONTROL_REG 0x07

class MotorDriver: public Plugin {

    std::string commandBuffer;
    std::queue<std::string> command_queue;
    char delimiter;
    std::shared_ptr<CommsBase> uartComms;
    uint32_t i2c_dev_addr;

    void parse_uartToCommand() {

        // Check if plugin exists
        if (!uartComms) {
            logger->log_error("MotorDriver", "UART plugin uninitialized.!\n");
        }

        // Read data from plugin
        std::vector<uint8_t> inputStream;
        size_t streamLen = 0;
        uartComms->read(0, inputStream, streamLen);
        std::string command(inputStream.begin(), inputStream.end());

        // Remove leading and trailing whitespaces
        command.erase(0, command.find_first_not_of(" "));
        command.erase(command.find_last_not_of(" ")+1);

        // Parse the string to create a command-queue
        std::istringstream iss(command);
        std::string token;

        std::getline(iss, token, delimiter);
        if (strcmp(token.c_str(),"MOTOR") != 0)
            return;
        while (std::getline(iss, token, delimiter)) {            
            command_queue.push(token);
        }
    }

    inline std::vector<uint8_t> motor_run(int16_t fr,int16_t fl ,int16_t rr,int16_t rl) {

        std::vector<uint8_t> speed(9);
        speed[0] = I2C_MOTOR_SPEED_CONTROL_REG;
        
        speed[1] = (fr>>8)&0xff;
        speed[2] = (fr)&0xff;
        
        speed[3] = (fl>>8)&0xff;
        speed[4] = (fl)&0xff;
        
        speed[5] = (rr>>8)&0xff;
        speed[6] = (rr)&0xff;
        
        speed[7] = (rl>>8)&0xff;
        speed[8] = (rl)&0xff;

        return speed;
    }

    void forward(int pulsecount, int speed) {
        commObj->write(i2c_dev_addr, motor_run(speed,speed,speed,speed));
    }

    void reverse(int pulsecount, int speed) {
        commObj->write(i2c_dev_addr, motor_run(-speed,-speed,-speed,-speed));
    }

    void left(int pulsecount, int speed) {
        commObj->write(i2c_dev_addr, motor_run(speed,-speed,speed,-speed));
    }


    void right(int pulsecount, int speed) {
        commObj->write(i2c_dev_addr, motor_run(-speed,speed,-speed,speed));
    }

    void stop() {
        commObj->write(i2c_dev_addr, motor_run(0,0,0,0));
    }

public:
    MotorDriver(std::string name, std::shared_ptr<CommsBase> comms_writer, std::shared_ptr<CommsBase> comms_reader, uint8_t i2c_addr, char cmd_delimiter = '\n') : Plugin(name, comms_writer), uartComms(comms_reader), i2c_dev_addr(i2c_addr), delimiter(cmd_delimiter) {}

    bool write_data(const std::vector<uint8_t>& buffer) { 
        commObj->write(i2c_dev_addr, buffer);
        return true; 
    }

    bool read_data(std::vector<uint8_t>& buffer) {
        
    }

    bool execute_action() override {
        
        // Check UART for motor command string
        parse_uartToCommand();
        if (command_queue.empty())
            return true;
        
        // Iterate through tokens
        while (!command_queue.empty()) {
            std::string cmdlet = command_queue.front();
            command_queue.pop();

            // Parse tokens
            char cmdChar; int pulseCount; int speed;
            int scanned = sscanf(cmdlet.c_str(), "%c %d %d", &cmdChar, &pulseCount, &speed);

            if (scanned != 3)
                return false;

            // Call command
            switch (toupper(cmdChar)) {
                case 'F': 
                    forward(pulseCount, speed);
                    return true;
                case 'B': 
                    reverse(pulseCount, speed);
                    return true;
                case 'L': 
                    left(pulseCount, speed);
                    return true;
                case 'R': 
                    right(pulseCount, speed);
                    return true;
                case 'S': 
                    stop();
                    return true;
                default:  
                    return true;
            }   

        }

        return true;
    }

};