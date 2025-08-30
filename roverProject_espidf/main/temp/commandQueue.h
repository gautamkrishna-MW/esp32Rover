#include <sstream>
#include <string>

#include <Arduino.h>
#include <queue>

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
    String serialBuffer;

public:
    void readFromSerial(Stream& serial) {
        while (serial.available()) {
            char c = serial.read();
            if (c == '\n') {
                parseAndQueueCommand(serialBuffer);
                serialBuffer = "";
            } else {
                serialBuffer += c;
            }
        }
    }

    void readFromString(std::string testInpStr) {
        std::stringstream ss(testInpStr);
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

private:
    void parseAndQueueCommandString(std::string line) {
        char cmdChar;
        int pulseCount;
        int speed;

        line.erase(0, line.find_first_not_of(' '));
        int scanned = sscanf(line.c_str(), "%c %d %d", &cmdChar, &pulseCount, &speed);
        if (scanned == 3) {
            Command cmd = parseCommand(cmdChar);
            if (cmd != Command::INVALID) {
                commandQueue.emplace(cmd, pulseCount, speed);
            }
        }
    }

    void parseAndQueueCommand(const String& line) {
        char cmdChar;
        int pulseCount;
        int speed;

        int scanned = sscanf(line.c_str(), "%c %d %d", &cmdChar, &pulseCount, &speed);
        if (scanned == 3) {
            Command cmd = parseCommand(cmdChar);
            if (cmd != Command::INVALID) {
                commandQueue.emplace(cmd, pulseCount, speed);
            }
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
