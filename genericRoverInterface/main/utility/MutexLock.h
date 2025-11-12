

#pragma once

#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class MutexLock {
    SemaphoreHandle_t mtx;
public:
    MutexLock(SemaphoreHandle_t& m) : mtx(m) {
        xSemaphoreTake(mtx, pdMS_TO_TICKS(100));
    }

    ~MutexLock() { 
        if (mtx) 
            xSemaphoreGive(mtx); 
    }
};