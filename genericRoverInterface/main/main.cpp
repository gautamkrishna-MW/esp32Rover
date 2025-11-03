#include <iostream>
#include <utility>

#include "rover/Rover.h"
#include "comms/Comms_UART.h"
#include "utility/Logger.h"

#include "plugin/Plugin_MotorDriver.h"

extern "C" {
    void app_main(void) {

        // Logger log_ptr("Rover");
        std::shared_ptr<Logger> log_ptr = std::make_shared<Logger>("Rover");
        std::unique_ptr<CommsBase> hostCommObj = std::make_unique<UARTComms>(log_ptr);

        std::string motorPluginName("Motor Driver");
        std::shared_ptr<Logger> motorLogger = std::make_shared<Logger>(motorPluginName);
        std::shared_ptr<CommsBase> commObj = nullptr;
        MotorPlugin motor(motorPluginName, commObj, motorLogger, '\n');

        Rover& rover_ptr = Rover::getInstance(hostCommObj, log_ptr);
        rover_ptr.init();
    }
}