/**
 * @file datamover_task.cpp
 * @brief FreeRTOS data mover task that drains AFE data and queues BLE TX.
 */

#include "sensors/core/datamover_task.h"

#include "drivers/ad5940_hal.h"
#include "sensors/core/sensor_manager.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

#include <Arduino.h>
#include <atomic>

namespace sensor {
  TaskHandle_t dataTaskHandle = nullptr;
  std::atomic_bool dataTaskStopRequested{false};
  constexpr TickType_t kDataMoverStopTimeoutTicks = pdMS_TO_TICKS(1000);
  void dataMoverTask(void* pvParameters);
}

void sensor::createDataMoverTask() {
  if (dataTaskHandle != nullptr) return;
  dataTaskStopRequested.store(false, std::memory_order_relaxed);

  xTaskCreate(dataMoverTask,    // Task function
              "Data Mover",     // Task name
              2048,             // Stack size (in words)
              nullptr,          // Task parameters
              1,                // Priority (very low)
              &dataTaskHandle); // Task handle
}

void sensor::stopDataMoverTask() {
  if (dataTaskHandle == nullptr) return;

  dataTaskStopRequested.store(true, std::memory_order_relaxed);
  if (!taskSync::requestStopAndWait(dataTaskHandle, kDataMoverStopTimeoutTicks))
    dbgWarn("Data mover task stop timed out");
}

void sensor::startDataMoverTask() {
  if (dataTaskHandle != nullptr) {
    BaseType_t taskWoken = pdFALSE;
    // This function is entered from the GPIO interrupt handler path. We must use
    // ISR-safe FreeRTOS APIs here; task-context APIs can assert or corrupt scheduler state.
    xTaskNotifyGiveFromISR(dataTaskHandle, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
  }
}

// This task is woken up if there is data to be grabbed from the AFE. It then
// grabs data from the AFE and pushes it to the respective sensor.
void sensor::dataMoverTask(void* pvParameters) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait until we are awaken by an interrupt
    if (dataTaskStopRequested.load(std::memory_order_relaxed)) break;

    dbgInfo("DataMover()");
    sensor::Sensor* activeSensor = getActiveSensor();
    if (activeSensor != nullptr) {
      activeSensor->ISR();
#ifdef DEBUG_MODE
      activeSensor->printResult();
#endif
      queueDataForTX(activeSensor->getNumBytesAvailable()); // Get the data and queue it up for transmitting
    }

    if (dataTaskStopRequested.load(std::memory_order_relaxed)) break;
  }

  dataTaskStopRequested.store(false, std::memory_order_relaxed);
  dataTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
