

#pragma once

#include <iostream>

#include "../utility/Message.h"
#include "../utility/Logger.h"
#include "../comms/CommsBase.h"

extern "C" {
    class Plugin {
    protected:
        std::shared_ptr<Logger> logger;
        std::string plugin_name;
        uint32_t plugin_id;
        std::shared_ptr<CommsBase> commObj;

        Message outputBuffer;
        std::vector<Message> msgBuffer;

    public:
        Plugin(std::string name, std::shared_ptr<CommsBase> comms, std::shared_ptr<Logger> logs) : 
        plugin_name(name), commObj(comms), logger(logs) {}

        virtual ~Plugin() = default;

        // ID Setter/Getter
        uint8_t get_id() {
            return plugin_id;
        }
        void set_id(uint32_t id) {
            plugin_id = id;
        }

        // Plugin name getter
        std::string getName() {
            return plugin_name;
        }

        // Set/get message buffer
        Message sendMessage() { 
            return outputBuffer;
        }
        void receiveMessage(Message msg) { 
            msgBuffer.push_back(msg);
        }
        std::vector<Message>* get_msgBuffer_ptr() {
            return &msgBuffer;
        }

        // Called once to setup plugin (pass platform comms if needed)
        virtual bool setup() {
            return true;
        }

        // Optional: Process incoming data
        virtual void process() = 0;

        // Optional: plugin can log its own status
        virtual void logStatus(std::string msg) const {
            logger->log_info("Status", msg.c_str());
        }
    };
}