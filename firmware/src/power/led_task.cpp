/**
 * @file led_task.cpp
 * @brief FreeRTOS LED indication task implementation.
 */

#include "power/led_task.h"

#include "HWConfig/constants.h"
#include "power/power.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

#include <Arduino.h>

namespace power {
  static TaskHandle_t heartbeatTaskHandle = nullptr;
  static void heartbeatTask(void* pvParameters);
  constexpr TickType_t kHeartbeatTaskStopTimeoutTicks = pdMS_TO_TICKS(1000);
}

void power::startHeartbeatTask() {
  stopHeartbeatTask();
  if (heartbeatTaskHandle != nullptr) {
    dbgError("LED heartbeat task did not stop cleanly");
    return;
  }

  BaseType_t rc = xTaskCreate(heartbeatTask,   // Task function
                              "LED Heartbeat", // Task name
                              512,             // Stack size (in words)
                              nullptr,         // Task parameters
                              0,               // Priority (very low)
                              &heartbeatTaskHandle
  );
  if (rc != pdPASS) dbgError("Failed to start LED heartbeat task");
}

void power::stopHeartbeatTask() {
  if (!taskSync::requestStopAndWait(heartbeatTaskHandle, kHeartbeatTaskStopTimeoutTicks))
    dbgWarn("LED heartbeat task stop timed out");
}

void power::heartbeatTask(void* pvParameters) {
  while (true) {
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;

    // LED Heartbeat ON
    pinMode(PIN_LED, OUTPUT); // Turn the pin back to an output (we move to an input to save power)
    digitalWrite(PIN_LED, HIGH);
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBlinkOn)) > 0) break;

    // LED Heartbeat OFF
    digitalWrite(PIN_LED, LOW);   // Unclear if this is needed, but it is a good practice
    disconnectInputGPIO(PIN_LED); // Disconnect the GPIO pin to save power
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBlinkOff)) > 0) break;
  }

  heartbeatTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
