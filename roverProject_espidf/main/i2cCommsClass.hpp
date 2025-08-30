
#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "utilities.hpp"

#define MAX_I2C_BUFFER_SIZE 32

class i2cCommClass {

private:
  uint8_t regDataArray[MAX_I2C_BUFFER_SIZE] = {0};
  uint8_t devAddress;

  i2c_master_bus_config_t bus_cfg = {};
  i2c_device_config_t dev_cfg = {};
  i2c_master_bus_handle_t busHandle = {};
  i2c_master_dev_handle_t devHandle = {};

public:
  i2cCommClass( uint8_t address): devAddress(address) {

    // I2C Bus configuration and setup
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = GPIO_NUM_21;
    bus_cfg.scl_io_num = GPIO_NUM_22;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    espErrChk(i2c_new_master_bus(&bus_cfg, &busHandle));

    // I2C Device configuration and setup
    dev_cfg.device_address = devAddress;
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.scl_speed_hz = 100000;
    espErrChk(i2c_master_bus_add_device(busHandle, &dev_cfg, &devHandle));

    ESP_LOGI(TAG, "I2C device Setup successful\n");
  }

  ~i2cCommClass() {
	  espErrChk(i2c_master_bus_rm_device(devHandle));
    espErrChk(i2c_del_master_bus(busHandle));
  }

  uint8_t* getBufferPtr() {
    return regDataArray;
  }

  uint8_t getDevAddress() {
    return devAddress;
  }

  void writeRegister(uint8_t regAddress, uint8_t regValue, int delayVal = 10) {
    // Setup I2C for writeing to specific address
    uint8_t buffer[2] = {0, 0};
    buffer[0] = regAddress;
    buffer[1] = regValue;
    espErrChk(i2c_master_transmit(devHandle, buffer, sizeof(buffer), -1));
  }

  void readData(uint8_t cmdRegister, uint8_t len = 1, int delayVal = 10) {
    espErrChk(i2c_master_transmit_receive(devHandle, &cmdRegister, 1, &regDataArray[0], len, -1));
  }
};