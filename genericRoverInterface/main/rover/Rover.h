

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <map>
#include <utility>
#include <queue>
#include <algorithm>

#include "../utility/Message.h"
#include "../utility/Logger.h"
#include "../comms/CommsBase.h"
#include "../plugin/Plugin.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define DEFAULT_TASK_STACK_SIZE 2048
#define INCLUDE_vTaskDelete 1
#define MESSAGE_BUFFER_SIZE 100
#define MESSAGE_QUEUE_WAIT_MS 100

using vect8 = std::vector<uint8_t>;

extern "C" {

    // Singleton rover class
    class Rover {
    private:

        // Singleton instance pointer
        inline static std::unique_ptr<Rover> instance;

        // Host communications
        std::unique_ptr<CommsBase> hostComm_;
        std::mutex mutex;

        // Plugin maps and handlers
        std::map<std::string, std::shared_ptr<Plugin>> pluginMap;
        std::map<std::string, TaskHandle_t> pluginHandleMap;
        std::shared_ptr<Logger> log;
        
        // Messages stack from plugins
        std::queue<Message> msgQueue;
        inline static std::mutex msgQMutex;
        QueueHandle_t msgQ;

        // Message-handler task handles
        TaskHandle_t msgTaskHandle = NULL;
        TaskHandle_t hostMsgTaskHandle = NULL;
        TaskHandle_t pluginMsgTaskHandle = NULL;

        Rover(std::unique_ptr<CommsBase> hostComm, std::shared_ptr<Logger>& logger)
            : hostComm_(std::move(hostComm)), log(logger) {
                log->log_info("Rover", "Rover instance created.\n");
            }

        // Helper: Byte stream to message queue
        static void parseByteStreamToMessage(vect8& buffer, QueueHandle_t& msgQ) {
            // Check the start byte
            uint8_t start_byte = Message::START_BYTE;
            auto startIter = std::find(buffer.begin(), buffer.end(), start_byte);
            if (startIter == buffer.end()) {
                return;
            }

            // If buffer is less than minimum size, discard
            uint32_t len = *(startIter + 1);
            if (len > buffer.size()) {
                buffer.clear();
                return;
            }

            // Remove preceding noise bytes
            buffer.erase(buffer.begin(), startIter);

            // If buffer is single frame, process it.
            len = *(startIter + 1);
            if (buffer.size() == len+2) {
                Message msg;
                Message::fromFrame(buffer, msg);
                xQueueSend(msgQ, (void*)&msg, MESSAGE_QUEUE_WAIT_MS/portTICK_PERIOD_MS);
                return;
            }
            else {
                // Iterate through buffer and create messages from frames
                while (!buffer.empty()) {
                    Message msg;
                    startIter = std::find(buffer.begin(), buffer.end(), start_byte);
                    len = *(startIter + 1);
                    vect8 frame(startIter, buffer.begin() + len + 2);
                    Message::fromFrame(frame, msg);
                    xQueueSend(msgQ, (void*)&msg, MESSAGE_QUEUE_WAIT_MS/portTICK_PERIOD_MS);
                    buffer.erase(buffer.begin(), startIter);
                }                
            }
        }

        // Task_function: Host Message Handler
        static void hostMessageHandler(void* params) {
            // Input is the self-pointer to Rover
            Rover* roverPtr = static_cast<Rover*>(params);
            
            // Iterate through host comms buffer and parse messages
            while(true) {
                // Lock the queue resource until the queue is filled
                vect8 buffer;
                size_t readLen = 0;
                roverPtr->hostComm_->read(0, buffer, readLen);
                if (!buffer.empty())
                    parseByteStreamToMessage(buffer, roverPtr->msgQ);
                
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        static void pluginMessageHandler(void* params) {
            // Input is the self-pointer to Rover
            Rover* roverPtr = static_cast<Rover*>(params);
            
            // Iterate through plugin's message buffer and queue messages
            while(true) {
                for (auto& kv : roverPtr->pluginMap) {
                    std::queue<Message> outMessages = kv.second->get_outMsgBuffer_ptr();
                    while (!outMessages.empty()) {
                        xQueueSend(roverPtr->msgQ, &(outMessages.front()), MESSAGE_QUEUE_WAIT_MS/portTICK_PERIOD_MS);
                        outMessages.pop();
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        // Task_function: Message handler.
        static void messageRouter(void* params) {
            // Input is the self-pointer to Rover
            Rover* roverPtr = static_cast<Rover*>(params);

            // Using task-queue, route messages accordingly.
            while(true) {
                Message msg;
                xQueueReceive(roverPtr->msgQ, &msg, MESSAGE_QUEUE_WAIT_MS/portTICK_PERIOD_MS);
                if (msg.dst != msg.src) {
                    // Check if message is for host or plugin
                    if (msg.dst == Message::HOST_ID) {
                        bool success = roverPtr->hostComm_->write(0, msg.toFrame());
                        roverPtr->log->espAssert(success);
                    }
                    else {
                        for (auto& kv : roverPtr->pluginMap) {
                            if (kv.second->get_id() == msg.dst) {
                                kv.second->receiveMessage(msg);
                            }
                        }
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        // Task_function: Plugin process.
        static void callPluginProcess(void* param) {
            // Get rover and plugin pointers
            Plugin* pluginPtr = static_cast<Plugin*>(param);
            if (!pluginPtr)
                pluginPtr->logStatus("Bad Plugin initialization");

            // Loop through the plugin process-function infinitely.
            while(true) {
                // Call the process function
                pluginPtr->process();
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

    public:
        
        // Pointer to singleton class
        static Rover& getInstance(std::unique_ptr<CommsBase>& hostComm, std::shared_ptr<Logger>& logger) {
            if (instance == nullptr) {
                instance.reset(new Rover(std::move(hostComm), logger));
            }
            return *instance;
        }
        ~Rover() { stop(); }

        // Prevent copying and assignment
        Rover(const Rover&) = delete;
        Rover& operator=(const Rover&) = delete;

        void registerPlugin(std::shared_ptr<Plugin> plugin, uint32_t id) {
            std::lock_guard<std::mutex> lk(mutex);
            std::string pStr = plugin->getName();
            pluginMap[pStr] = plugin;
            pluginMap[pStr]->set_id(id);

            std::string msg = "Registered plugin " + plugin->getName();
            log->log_info("Rover", msg.c_str());
        }

        bool init() {
            if (!hostComm_) {
                log->log_error("Rover", "No hostComm set");
                return false;
            }
            if (!hostComm_->open(0)) {
                log->log_error("Rover", "Host comm unable to open");
                return false;
            }
            // setup each plugin
            for (auto& kv : pluginMap) {
                if (!kv.second->setup()) {
                    log->log_info("Rover", "Plugin setup failed: %s", kv.second->getName());
                }
            }

            msgQ = xQueueCreate(MESSAGE_BUFFER_SIZE, sizeof(Message));
            assert(msgQ);
            // log->print_msg("Rover", "Q: %d\n", msgQ);
            // log->espAssert(msgQ != NULL, __FILE__, __LINE__);

            // Create task for message routing
            xTaskCreate(messageRouter, "Message Handler", DEFAULT_TASK_STACK_SIZE, this, 2, &msgTaskHandle);
            assert(msgTaskHandle);

            // Create task for receiving messages from host
            xTaskCreate(hostMessageHandler, "Host Message Handler", DEFAULT_TASK_STACK_SIZE, this, 2, &hostMsgTaskHandle);
            assert(hostMsgTaskHandle);

            // Create task for receiving messages from plugins
            xTaskCreate(pluginMessageHandler, "Plugin Message Handler", DEFAULT_TASK_STACK_SIZE, this, 2, &pluginMsgTaskHandle);
            assert(pluginMsgTaskHandle);

            // Create task for each plugin (all plugins run concurrently and in parallel)
            for (auto& kv : pluginMap) {
                TaskHandle_t taskHandle = NULL;
                xTaskCreate(callPluginProcess, 
                kv.second->getName().c_str(), DEFAULT_TASK_STACK_SIZE, (void*)&kv.second, 1, &taskHandle);
                assert(taskHandle);
                pluginHandleMap[kv.second->getName()] = taskHandle;
            }

            log->log_info("Rover", "Initialization successful.\n");
            return true;
        }

        void stop() {
            for (auto& kv : pluginHandleMap) {
                vTaskDelete(kv.second);
            }
            vTaskDelete(msgTaskHandle);
            vTaskDelete(hostMsgTaskHandle);
            instance = nullptr;
        }
    };
}