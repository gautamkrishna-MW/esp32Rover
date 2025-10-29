
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <mutex>

#include "CommsBase.h"

#include "driver/uart.h"
#include "driver/gpio.h"

extern "C" {

    // --- UART Comms Example ---
    class UARTComms : public CommsBase {
    private:
        int baud_rate;
        uint32_t uart_buffer_size;
        uart_port_t uart_port;
        
        static std::mutex mtx_lock;

    public:
        UARTComms(std::shared_ptr<Logger> logger, uart_port_t uart_dev = UART_NUM_0, gpio_num_t tx_pin = GPIO_NUM_1, gpio_num_t rx_pin = GPIO_NUM_3, int baudrate = 115200, uint32_t buffersize = 2048, uart_word_length_t databits = UART_DATA_8_BITS, uart_parity_t parity = UART_PARITY_DISABLE, uart_stop_bits_t stopbits = UART_STOP_BITS_1, uart_hw_flowcontrol_t flw_ctrl = UART_HW_FLOWCTRL_DISABLE, uart_sclk_t clk_freq = UART_SCLK_DEFAULT): CommsBase(logger), baud_rate(baudrate), uart_buffer_size(buffersize), uart_port(uart_dev) {

            // Setup UART device and buffer size
            uart_config_t uart_config_struct = {};        
            
            // UART Config
            uart_config_struct.baud_rate = baud_rate;
            uart_config_struct.data_bits = databits;
            uart_config_struct.parity    = parity;
            uart_config_struct.stop_bits = stopbits;
            uart_config_struct.flow_ctrl = flw_ctrl;
            uart_config_struct.source_clk = clk_freq;

            // Install driver and setup UART
            comms_logger->espErrChk(uart_set_hw_flow_ctrl(uart_port, flw_ctrl, 0));
            comms_logger->espErrChk(uart_param_config(uart_port, &uart_config_struct));
            comms_logger->espErrChk(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

            comms_logger->log_info("UART", "UART Comms setup successful.\n");
        }

        bool open(uint32_t dev_addr) override {
            // open UART port
            comms_logger->espErrChk(uart_driver_install(uart_port, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
            return true;
        }

        bool read(uint32_t dev_addr, std::vector<uint8_t>& buffer, size_t len) override {
            if (len > uart_buffer_size-1) {
                comms_logger->log_error("UART", "UART Buffer length exceeded!\n");
                return false;
            }
            
            buffer.resize(uart_buffer_size);
            {
                // UART Read critical section
                comms_logger->log_info("UART", "Locking the bus for read. \n");
                std::lock_guard<std::mutex> lock(mtx_lock);
                len = uart_read_bytes(uart_port, (void*) buffer.data(), len,  20 / portTICK_PERIOD_MS);
            }
            buffer.resize(len);
            comms_logger->log_info("UART", "UART read successful.\n");
            return true;
        }

        bool write(uint32_t dev_addr, const std::vector<uint8_t>& buffer) override {
            {
                // UART Write critical section
                comms_logger->log_info("UART", "Locking the bus for write. \n");
                std::lock_guard<std::mutex> lock(mtx_lock);
                uart_write_bytes(uart_port, buffer.data(), buffer.size());
            }
            comms_logger->log_info("UART", "UART write successful.\n");
            return true;
        }

        ~UARTComms() {
            uart_driver_delete(uart_port);
            comms_logger->log_info("UART", "UART resource released. Closing UART.! \n");
        }
    };
}