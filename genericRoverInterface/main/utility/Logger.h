
#pragma once

#include "esp_log.h"
#include "esp_err.h"
#include <string>

extern "C" {
    class Logger {
    protected:
        std::string plugin_name;

    public:
        Logger(std::string tag): plugin_name(tag) {}

        void setLogLevel(esp_log_level_t log_level) {
            esp_log_level_set(plugin_name.c_str(), log_level);
        }

        inline void espAssert(esp_err_t code, const char *file = __FILE__, int line = __LINE__, bool abort=true) {
            if (code != ESP_OK) {
                ESP_LOGE(plugin_name.c_str(), "ESPAssert: %s %s %d\n", esp_err_to_name(code), file, line);
                if (abort) 
                    exit(code);
            }
        }

        void espErrChk(esp_err_t err_code, bool abort=true) {
            espAssert(err_code, __FILE__, __LINE__, abort);
            printf("\n");
        }

        void log_info(const char* prefix, const char *format, ...) {
            std::string prefixMsg = plugin_name + ":" + prefix + " ";
            va_list args;
            va_start(args, format);
            esp_log_writev(ESP_LOG_INFO, prefixMsg.c_str(), format, args);
            va_end(args);        
        }

        void log_error(const char* prefix, const char *format, ...) {
            std::string prefixMsg = plugin_name + ":" + prefix + " ";
            va_list args;
            va_start(args, format);
            esp_log_writev(ESP_LOG_ERROR, prefixMsg.c_str(), format, args);
            va_end(args);        
        }

        void print_msg(const char* prefix, const char *format, ...) {
            std::string prefixMsg = plugin_name + ":" + prefix + " ";
            va_list args;
            va_start(args, format);
            printf("[CUSTOM MSG] %s: ", prefixMsg.c_str());
            vprintf(format, args);
            va_end(args);
            printf("\n");
        }
    };
}