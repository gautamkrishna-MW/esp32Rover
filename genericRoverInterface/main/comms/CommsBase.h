
#pragma once
#include <iostream>
#include <vector>
#include <memory>

#include "../utility/Logger.h"

extern "C" {
    // --- Base CommsBase Interface ---
    class CommsBase {
    protected:
        std::shared_ptr<Logger> comms_logger;

    public:
        CommsBase(std::shared_ptr<Logger> logger) : comms_logger(logger) {}

        virtual bool open(uint32_t dev_addr) { return true; }
        virtual bool close() { return true; }

        virtual bool read(uint32_t dev_addr, std::vector<uint8_t>& buffer, size_t len) { return false; }
        virtual bool write(uint32_t dev_addr, const std::vector<uint8_t>& buffer) { return false; }

        virtual ~CommsBase() {}
    };
}