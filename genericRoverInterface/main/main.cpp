#include <iostream>
#include <utility>

#include "rover/Rover.h"
#include "comms/Comms_UART.h"
#include "utility/Logger.h"

#include "plugin/Plugin_MotorDriver.h"

#include "tests/test_plugin_Talk.h"

extern "C" {
    void app_main(void) {

        // Logger log_ptr("Rover");
        std::shared_ptr<Logger> log_ptr = std::make_shared<Logger>("Rover");
        std::unique_ptr<CommsBase> hostCommObj = std::make_unique<UARTComms>(log_ptr);
        std::shared_ptr<CommsBase> commObj = nullptr;
        // std::string motorPluginName("Motor Driver");
        // std::shared_ptr<Logger> motorLogger = std::make_shared<Logger>(motorPluginName);
        // MotorPlugin motor(motorPluginName, commObj, motorLogger, '\n');

        std::shared_ptr<Logger> rxLogger = std::make_shared<Logger>("Rx Logger");
        std::shared_ptr<test_receiver> rx = std::make_shared<test_receiver>("Test Receiver Plugin", commObj, rxLogger);

        std::shared_ptr<Logger> txLogger = std::make_shared<Logger>("Tx Logger");
        std::shared_ptr<test_sender> tx = std::make_shared<test_sender>("Test Transmitter Plugin", commObj, txLogger);

        log_ptr->setLogLevel(ESP_LOG_MAX);
        rxLogger->setLogLevel(ESP_LOG_MAX);
        txLogger->setLogLevel(ESP_LOG_MAX);

        Rover& rover_ptr = Rover::getInstance(hostCommObj, log_ptr);
        rover_ptr.registerPlugin(tx, 1);
        rover_ptr.registerPlugin(rx, 2);

        rover_ptr.init();
    }
}