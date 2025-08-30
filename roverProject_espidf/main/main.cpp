#include "utilities.hpp"
#include "roverControl.hpp"

extern "C" {

    void test_rover() {
        printf("Rover Test\n");

        std::unique_ptr<roverControl> rover = std::make_unique<roverControl>();
        while (1) {
            rover->run();
            vTaskDelay(500/portTICK_PERIOD_MS);
        }
    }


    void app_main(void) {

        printf("Start Test\n");
        esp_log_level_set(TAG, ESP_LOG_DEBUG);
        
        // test_motor();
        // test_orientation();
        // test_uart();

        test_rover();
    }
}