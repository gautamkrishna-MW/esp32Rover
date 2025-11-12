
#pragma once

#include <iostream>
#include <string>
#include "../plugin/Plugin.h"

extern "C" {
    class test_sender : public Plugin {
    public:
        test_sender(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> logs) : Plugin(name, comms, logs) {
            logStatus("Created sender plugin.\n");
        }

        ~test_sender() {}

        void process() {
            std::string sendString("Hello Rover, sending message\n");
            std::vector<uint8_t> msgStr(sendString.begin(), sendString.end());            
            Message testMsg(1,2,0,msgStr);
            sendMessage(testMsg);
        }
    };

    class test_receiver : public Plugin {
    public:
        test_receiver(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> logs) : Plugin(name, comms, logs) {
            logStatus("Created receiver plugin.\n");
        }

        ~test_receiver() {}

        void process() {
            if (!msgBuffer.empty()) {
                for (int i=0; i<msgBuffer.size(); i++) {
                    std::string msg(msgBuffer[i].payload.begin(),msgBuffer[i].payload.end());
                    msg += " Received!!\n";
                    logStatus(msg);
                }
                msgBuffer.clear();
            }
        }
    };
}