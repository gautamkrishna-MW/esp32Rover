
#pragma once

#include <iostream>
#include <string>
#include "../plugin/Plugin.h"

extern "C" {
    class test_uartSender : public Plugin {
    public:
        test_uartSender(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> logs) : Plugin(name, comms, logs) {
            logStatus("Created sender plugin.\n");
        }

        ~test_uartSender() {}

        void process() {
            std::string sendString("Hello Rover, sending message\n");
            std::vector<uint8_t> msgStr(sendString.begin(), sendString.end());            
            Message testMsg(1,2,0,msgStr);
            sendMessage(testMsg);
        }
    };

    class test_uartReceiver : public Plugin {
    public:
        test_uartReceiver(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> logs) : Plugin(name, comms, logs) {
            logStatus("Created receiver plugin.\n");
        }

        ~test_uartReceiver() {}

        void process() {
            if (!msgBuffer.empty()) {
                std::string sendString("Hello Rover, received message\n");
                for (int i=0; i<msgBuffer.size(); i++) {
                    std::string msg(msgBuffer[i].payload.begin(),msgBuffer[i].payload.end());
                    logStatus(msg);
                }
            }
        }
    };
}