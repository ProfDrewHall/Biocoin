// Module: FreeRTOS task shutdown helpers.
// Purpose: Provide cooperative stop/wait behavior for long-running worker tasks.

#pragma once

#include <Arduino.h>

namespace taskSync {
  /// Request a task stop via notification and wait for its handle to clear.
  /// @param taskHandle Task handle reference to stop.
  /// @param timeoutTicks Max wait time in FreeRTOS ticks.
  /// @return True if task exited before timeout, false otherwise.
  inline bool requestStopAndWait(TaskHandle_t& taskHandle, TickType_t timeoutTicks) {
    if (taskHandle == nullptr) return true;

    xTaskNotifyGive(taskHandle);

    TickType_t start = xTaskGetTickCount();
    while (taskHandle != nullptr) {
      vTaskDelay(pdMS_TO_TICKS(1));
      if ((xTaskGetTickCount() - start) > timeoutTicks) return false;
    }

    return true;
  }
} // namespace taskSync
