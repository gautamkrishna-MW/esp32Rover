
#pragma once

#include "utilities.hpp"
#include "driver/uart.h"

class uartCommsClass {

private:
    int uart_buffer_size{};
    uart_config_t uart_config_struct = {};
    
public:
    uartCommsClass() {
        // Set UART buffer size
        uart_buffer_size = UART_BUFF_SIZE;
        
        // UART Config
        uart_config_struct.baud_rate = UART_BAUD_RATE;
        uart_config_struct.data_bits = UART_DATA_8_BITS;
        uart_config_struct.parity    = UART_PARITY_DISABLE;
        uart_config_struct.stop_bits = UART_STOP_BITS_1;
        uart_config_struct.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config_struct.source_clk = UART_SCLK_DEFAULT;
        
        // Install driver and setup UART
        espErrChk(uart_set_hw_flow_ctrl(UART_NUM_0, UART_HW_FLOWCTRL_DISABLE, 0));
        espErrChk(uart_param_config(UART_NUM_0, &uart_config_struct));
        espErrChk(uart_set_pin(UART_NUM_0, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        espErrChk(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
        ESP_LOGI(TAG,"UART Setup Successful\n");
    }

    ~uartCommsClass() {
        uart_driver_delete(UART_NUM_0);
        ESP_LOGI(TAG, "Cleaning: UART\n");
    }

    size_t uart_write(const void* buffer, size_t len) {
        return uart_write_bytes(UART_NUM_0, buffer, len);
    }

    size_t uart_read(uint8_t* buffer, size_t len) {
        if (len > UART_BUFF_SIZE-1) {
            ESP_LOGE(TAG, "UART Buffer length exceeded!\n");
            return 0;
        }
        return uart_read_bytes(UART_NUM_0, (void*) buffer, len,  20 / portTICK_PERIOD_MS);
    }

    void uart_print(const char* inp) {
        uart_write_bytes(UART_NUM_0, inp, strlen(inp));
    }
};