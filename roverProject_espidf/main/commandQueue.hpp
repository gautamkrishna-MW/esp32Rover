
#pragma once

#include <sstream>
#include <string>
#include <queue>
#include "utilities.hpp"

enum class Command { FORWARD, BACKWARD, LEFT, RIGHT, STOP, INVALID };

struct RoverCommand {
    Command cmd;
    int pulseCount;
    uint8_t speed;  // 0–100

    RoverCommand(Command d = Command::INVALID, int p = 0, uint8_t s = 0)
        : cmd(d), pulseCount(p), speed(s) {}
};

class RoverCommandQueue {
private:
    std::queue<RoverCommand> commandQueue;
    std::string serialBuffer;

public:
    void readFromString(std::string inputStr) {
        std::stringstream ss(inputStr);
        std::string token;
        while(std::getline(ss, token, '\n')) {
            parseAndQueueCommandString(token.c_str());
        }
    }

    bool hasCommand() const {
        return !commandQueue.empty();
    }

    RoverCommand getNextCommand() {
        if (hasCommand()) {
            RoverCommand cmd = commandQueue.front();
            commandQueue.pop();
            return cmd;
        }
        return RoverCommand();  // Invalid
    }

    void clear() {
        while (!commandQueue.empty()) commandQueue.pop();
    }

    void parseAndQueueCommandString(std::string line) {
        char cmdChar;
        int pulseCount;
        int speed;
        Command cmd;

        line.erase(0, line.find_first_not_of(' '));
        int scanned = sscanf(line.c_str(), "%c %d %d", &cmdChar, &pulseCount, &speed);
        if (scanned == 3) {
            cmd = parseCommand(cmdChar);
        }
        else if (scanned == 2) {
            cmd = parseCommand(cmdChar);
            speed = 50; // Set default to 50
        }

        // Chec command validity
        if (cmd != Command::INVALID) {
            commandQueue.emplace(cmd, pulseCount, speed);
        }
        else {
            ESP_LOGE(TAG, "Invalid command received\n");
        }
    }

    Command parseCommand(char cmdChar) {
        switch (toupper(cmdChar)) {
            case 'F': return Command::FORWARD;
            case 'B': return Command::BACKWARD;
            case 'L': return Command::LEFT;
            case 'R': return Command::RIGHT;
            case 'S': return Command::STOP;
            default:  return Command::INVALID;
        }
    }
};
