

#pragma once
#include <iostream>
#include <vector>
#include <mutex>
#include <map>

#include "CommsBase.h"

// --- I2C Comms Example ---
class I2CComms : public CommsBase {
private:

    static std::mutex mtx_lock;    

    // I2C Bus
    i2c_port_num_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    i2c_clock_source_t clock_source;
    uint8_t glitch_ignore_count;
    uint32_t enable_internal_pullup;
    i2c_master_bus_handle_t busHandle = {};

    // I2C Device
    std::map<uint8_t, i2c_master_dev_handle_t> device_list;

public:
    I2CComms(std::shared_ptr<LoggerBase> logger, i2c_port_num_t port = I2C_NUM_0, gpio_num_t data_pin = GPIO_NUM_21, gpio_num_t clk_pin = GPIO_NUM_22, i2c_clock_source_t clk_source = I2C_CLK_SRC_DEFAULT, uint8_t glitch_count = 7, uint32_t internal_pullup = true) :
    CommsBase(logger),
    i2c_port(port),
    sda_pin(data_pin),
    scl_pin(clk_pin),
    clock_source(clk_source),
    glitch_ignore_count(glitch_count),
    enable_internal_pullup(internal_pullup) {

        // I2C Bus configuration and setup
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = i2c_port;
        bus_cfg.sda_io_num = sda_pin;
        bus_cfg.scl_io_num = scl_pin;
        bus_cfg.clk_source = clock_source;
        bus_cfg.glitch_ignore_cnt = glitch_ignore_count;
        bus_cfg.flags.enable_internal_pullup = enable_internal_pullup;
        comms_logger->espErrChk(i2c_new_master_bus(&bus_cfg, &busHandle));

        comms_logger->log_info("I2C", "I2C Comms setup successful.\n");
    }

    bool open(uint32_t dev_addr) override {
        // Check if device already exists, if not, add the device.
        auto it = device_list.find(static_cast<uint16_t>(dev_addr));
        if (it == device_list.end()){
            // I2C Device configuration and setup
            i2c_master_dev_handle_t devHandle = {};
            i2c_device_config_t dev_cfg = {};
            dev_cfg.device_address = static_cast<uint16_t>(dev_addr);
            dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_cfg.scl_speed_hz = 100000;
            comms_logger->espErrChk(i2c_master_bus_add_device(busHandle, &dev_cfg, &devHandle));

            // Add to list of devices
            device_list[dev_cfg.device_address] = devHandle;
            comms_logger->log_info("I2C", "Identified new I2C device on the bus. \n");
        }
        comms_logger->log_info("I2C", "I2C Comms port opened. \n");
        return true;
    }

    bool read(uint32_t dev_addr, std::vector<uint8_t>& buffer, size_t& len) override {

        uint8_t register_address = buffer[0];
        // Lock the bus when multiple device wants to read/write
        buffer.resize(len);
        {
            // Locking critical section
            comms_logger->log_info("I2C", "Locking the bus for read. \n");
            std::lock_guard<std::mutex> lock(mtx_lock);
            comms_logger->espErrChk(i2c_master_transmit_receive(device_list[static_cast<uint16_t>(dev_addr)], &register_address, 1, buffer.data(), len, -1));
        }
        comms_logger->log_info("I2C", "Data read complete. \n");
        return true;
    }

    bool write(uint32_t dev_addr, const std::vector<uint8_t>& buffer) override {
        {
            // Lock the bus when multiple device wants to read/write
            comms_logger->log_info("I2C", "Locking the bus for write. \n");
            std::lock_guard<std::mutex> lock(mtx_lock);
            comms_logger->espErrChk(i2c_master_transmit(device_list[static_cast<uint16_t>(dev_addr)], buffer.data(), buffer.size(), -1));
        }
        comms_logger->log_info("I2C", "Data write complete. \n");
        return true;
    }

    ~I2CComms(){
        for(auto& devHandle: device_list)
            comms_logger->espErrChk(i2c_master_bus_rm_device(devHandle.second));
	    comms_logger->espErrChk(i2c_del_master_bus(busHandle));
        comms_logger->log_info("I2C", "Devices and bus resources released. Closing I2C.! \n");
    }
};