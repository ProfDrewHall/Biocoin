#include "sensors/datamover_task.h"

#include "util/debug_log.h"
#include "drivers/ad5940_hal.h"
#include "sensors/SensorManager.h"
#include "util/task_sync.h"

#include <Arduino.h>

namespace sensor {
  TaskHandle_t dataTaskHandle = nullptr;
  bool dataTaskStopRequested = false;
  constexpr TickType_t kDataMoverStopTimeoutTicks = pdMS_TO_TICKS(1000);
  void dataMoverTask(void* pvParameters);
}

void sensor::createDataMoverTask() {
  if (dataTaskHandle != nullptr) return;
  dataTaskStopRequested = false;

  xTaskCreate(dataMoverTask,    // Task function
              "Data Mover",     // Task name
              2048,             // Stack size (in words)
              nullptr,          // Task parameters
              1,                // Priority (very low)
              &dataTaskHandle); // Task handle
}

void sensor::stopDataMoverTask() {
  if (dataTaskHandle == nullptr) return;

  dataTaskStopRequested = true;
  vTaskResume(dataTaskHandle);
  if (!taskSync::requestStopAndWait(dataTaskHandle, kDataMoverStopTimeoutTicks)) dbgWarn("Data mover task stop timed out");
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
    if (dataTaskStopRequested) break;

    dbgInfo("DataMover()");
    sensor::Sensor* activeSensor = getActiveSensor();
    if (activeSensor != nullptr) {
      activeSensor->ISR();
      #ifdef DEBUG_MODE
        activeSensor->printResult();
      #endif
      queueDataForTX(activeSensor->getNumBytesAvailable());       // Get the data and queue it up for transmitting
    }

    if (dataTaskStopRequested) break;
    vTaskSuspend(nullptr); // suspend until re-notified
  }

  dataTaskStopRequested = false;
  dataTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
