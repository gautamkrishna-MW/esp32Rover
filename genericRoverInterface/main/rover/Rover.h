

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

#define DEFAULT_TASK_STACK_SIZE 2048
#define INCLUDE_vTaskDelete 1

using vect8 = std::vector<uint8_t>;

extern "C" {

    class Rover {
    private:

        // Plugin maps and handlers
        std::map<std::string, std::shared_ptr<Plugin>> pluginMap;
        std::map<std::string, TaskHandle_t> pluginHandleMap;
        Logger log;
        static uint32_t plugin_id;
        
        // Host communications
        std::unique_ptr<CommsBase> hostComm_;
        std::mutex mutex;

        // Messages stack from plugins
        std::queue<Message> msgQueue;
        static std::mutex msgQMutex;

        // Message-handler task handles
        TaskHandle_t msgTaskHandle = NULL;
        TaskHandle_t hostMsgTaskHandle = NULL;

        // Helper: Byte stream to message queue
        static bool parseByteStreamToMessage(vect8 buffer, std::queue<Message>& msgQ) {
            // Check the start byte
            uint8_t start_byte = Message::START_BYTE;
            auto startIter = std::find(buffer.begin(), buffer.end(), start_byte);
            if (startIter == buffer.end()) {
                return false;
            }

            // If buffer is less than minimum size, discard
            uint32_t len = *(startIter + 1);
            if (len > buffer.size()) {
                buffer.clear();
                return false;
            }

            // Remove preceding noise bytes
            buffer.erase(buffer.begin(), startIter);

            // If buffer is single frame, process it.
            uint32_t len = *(startIter + 1);
            if (buffer.size() == len+2) {
                Message msg;
                Message::fromFrame(buffer, msg);
                msgQ.push(msg);
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
                    msgQ.push(msg);
                    buffer.erase(buffer.begin(), startIter);
                }                
            }
        }

        // Task_function: Host Message Handler
        static void hostMessageHandler(void* params) {
            // Input is the self-pointer to Rover
            Rover* roverPtr = static_cast<Rover*>(params);
            
            // Create lock and intiate the polling process
            std::unique_lock<std::mutex> mtxlock(msgQMutex, std::defer_lock);
            while(true) {
                // Lock the queue resource until the queue is filled
                vect8 buffer;
                size_t readLen = 0;
                mtxlock.lock();
                roverPtr->hostComm_->read(0, buffer, readLen);
                parseByteStreamToMessage(buffer, roverPtr->msgQueue);
                mtxlock.unlock();                
            }
        }

        // Task_function: Message handler.
        static void messageHandler(void* params) {
            // Input is the self-pointer to Rover
            Rover* roverPtr = static_cast<Rover*>(params);

            // Create lock and intiate the polling process
            std::unique_lock<std::mutex> mtxlock(msgQMutex, std::defer_lock);
            while(true) {
                // Check if queue is empty
                if (!roverPtr->msgQueue.empty()) {
                    // Lock the queue resource until the queue is empty
                    mtxlock.lock();
                    while (!roverPtr->msgQueue.empty()) {
                        // Iterate over messages and transmit them
                        Message& msg = roverPtr->msgQueue.front();
                        if (msg.dst != msg.src) {
                            // Check if message is for host or plugin
                            if (msg.dst == Message::HOST_ID) {
                                bool success = roverPtr->hostComm_->write(0, msg.toFrame());
                                roverPtr->log.espAssert(success);
                            }
                            else {
                                for (auto& kv : roverPtr->pluginMap) {
                                    if (kv.second->get_id() == msg.dst) {
                                        kv.second->receiveMessage(msg);
                                        break;
                                    }
                                }
                            }
                        }
                        // Pop messages after sending
                        roverPtr->msgQueue.pop();
                    }
                    mtxlock.unlock();
                }
            }
        }

        // Task_function: Plugin process.
        static void callPluginProcess(void* param) {
            // Cast pair back to corresponding pointers
            std::pair<Rover*, Plugin*>* taskArg = static_cast<std::pair<Rover*, Plugin*>*>(param);
            
            // Get rover and plugin pointers
            Rover* roverPtr = static_cast<Rover*>(taskArg->first);
            Plugin* pluginPtr = static_cast<Plugin*>(taskArg->second);
            if (!pluginPtr)
                pluginPtr->logStatus("Bad Plugin initialization");

            // Loop through the plugin process-function infinetly.
            while(true) {
                // Call the process function
                pluginPtr->process();
                // Check outgoing messages and stack
                Message out;
                if (!pluginPtr->sendMessage().isempty()) {
                    if (out.dst == Message::HOST_ID)
                        roverPtr->msgQueue.push(out);
                    else
                        roverPtr->msgQueue.push(out);
                }
            }
        }

    public:
        Rover(std::unique_ptr<CommsBase> hostComm, Logger& logger)
            : hostComm_(std::move(hostComm)), log(logger) {
                plugin_id = 0;
            }

        ~Rover() { stop(); }

        void registerPlugin(std::shared_ptr<Plugin> plugin) {
            std::lock_guard<std::mutex> lk(mutex);
            std::string pStr = plugin->getName();
            pluginMap[pStr] = plugin;
            pluginMap[pStr]->set_id(plugin_id++);

            std::string msg = "Registered plugin " + plugin->getName();
            log.log_info("Rover", msg.c_str());
        }

        bool init() {
            if (!hostComm_) {
                log.log_error("Rover", "No hostComm set");
                return false;
            }
            if (!hostComm_->open()) {
                log.log_error("Rover", "Host comm unable to open");
                return false;
            }
            // setup each plugin
            for (auto& kv : pluginMap) {
                if (!kv.second->setup()) {
                    log.log_info("Rover", "Plugin setup failed: %s", kv.second->getName());
                }
            }

            // Create task for each plugin (all plugins run concurrently and in parallel)
            TaskHandle_t taskHandle = NULL;
            for (auto& kv : pluginMap) {
                std::pair<Rover*, Plugin*> taskArg = std::make_pair<Rover*, Plugin*>(this, kv.second.get());
                xTaskCreate(callPluginProcess, 
                kv.second->getName().c_str(), DEFAULT_TASK_STACK_SIZE, (void*)&taskArg, 1, &taskHandle);
                log.espAssert(taskHandle != NULL);
                pluginHandleMap[kv.second->getName()] = taskHandle;
            }
            
            // Create task for message routing
            xTaskCreate(messageHandler, "Message Handler", DEFAULT_TASK_STACK_SIZE, this, 2, &msgTaskHandle);
            log.espAssert(msgTaskHandle != NULL);

            // Create task for receiving messages from host
            xTaskCreate(hostMessageHandler, "Host Message Handler", DEFAULT_TASK_STACK_SIZE, this, 2, &hostMsgTaskHandle);
            log.espAssert(hostMsgTaskHandle != NULL);

            log.log_info("Rover", "Initialization successful.\n");
            return true;
        }

        void stop() {
            for (auto& kv : pluginHandleMap) {
                vTaskDelete(kv.second);
            }
            vTaskDelete(msgTaskHandle);
            vTaskDelete(hostMsgTaskHandle);
        }
    };
}