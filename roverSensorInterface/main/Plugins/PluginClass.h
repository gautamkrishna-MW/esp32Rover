
#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "../Comms/CommsBase.h"
#include "../Logger/LoggerBase.h"


class Plugin {
protected:
    std::shared_ptr<CommsBase> commObj;
    std::unique_ptr<LoggerBase> logger;
    std::string plugin_name;

public:
    Plugin(std::string name, std::shared_ptr<CommsBase> t) : plugin_name(name), commObj(t) { 
        logger = std::make_unique<LoggerBase>(plugin_name);
    }

    virtual bool init(esp_log_level_t log_level = ESP_LOG_INFO) { 

        // Set log Level and logger initial info
        logger->setLogLevel(log_level);
        logger->log_info("Initializing Plugin %s\n", plugin_name.c_str());
                
        return true; 
    }
    
    // Virtual read/write for uint8
    virtual bool read_data(std::vector<uint8_t>& buffer) { 
        return true; 
    }

    virtual bool write_data(const std::vector<uint8_t>& buffer) { 
        return true; 
    }

    virtual bool execute_action() {
        return false; // no actuator
    }

    virtual std::string name() const { return plugin_name; }
    virtual ~Plugin() {}
};

