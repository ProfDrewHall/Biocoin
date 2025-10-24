#include "sensors/datamover_task.h"

#include "HWConfig/constants.h"
#include "util/debug_log.h"
#include "drivers/AD5940_hal.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"

#include <Arduino.h>

using namespace sensor;

namespace sensor {
  TaskHandle_t dataTaskHandle = nullptr;  
  void dataMoverTask(void* pvParameters);
}

void sensor::createDataMoverTask() {
  xTaskCreate(dataMoverTask,    // Task function
              "Data Mover",     // Task name
              2048,             // Stack size (in words)
              nullptr,          // Task parameters
              1,                // Priority (very low)
              &dataTaskHandle); // Task handle
}

void sensor::startDataMoverTask() {
  if (dataTaskHandle != nullptr) {
    xTaskNotifyGive(dataTaskHandle);
    vTaskResume(dataTaskHandle);
  }
}

// This task is woken up if there is data to be grabbed from the AFE. It then
// grabs data from the AFE and pushes it to the respective sensor.
void sensor::dataMoverTask(void* pvParameters) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until we are awaken by an interrupt

    dbgInfo("DataMover()");
    if (pActiveSensor != nullptr) {
      pActiveSensor->ISR();
      #ifdef DEBUG_MODE
        pActiveSensor->printResult();
      #endif
      queueDataForTX(pActiveSensor->getNumBytesAvailable());       // Get the data and queue it up for transmitting
    }
    
    vTaskSuspend(nullptr); // suspend until re-notified
  }
}
