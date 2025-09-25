
#pragma once
#include <iostream>
#include <vector>
#include <memory>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../Logger/LoggerBase.h"

// --- Base CommsBase Interface ---
class CommsBase {
protected:
    std::shared_ptr<LoggerBase> comms_logger;

public:
    CommsBase(std::shared_ptr<LoggerBase> logger) : comms_logger(logger) {}

    virtual bool open(uint32_t dev_addr) { return true; }
    virtual bool close() { return true; }

    virtual bool read(uint32_t dev_addr, std::vector<uint8_t>& buffer, size_t& len) { return false; }
    virtual bool write(uint32_t dev_addr, const std::vector<uint8_t>& buffer) { return false; }

    virtual ~CommsBase() {}
};




