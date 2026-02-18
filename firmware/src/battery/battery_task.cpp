// Module: Battery monitor task.
// Purpose: Runs periodic battery sampling and publishes level via BLE BAS.
// Notes: Uses cooperative stop notification to avoid abrupt task deletion.

#include "battery/battery_task.h"

#include "HWConfig/config.h"
#include "battery/battery.h"
#include "bluetooth/bluetooth.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

#include <Arduino.h>

namespace battery {
  static TaskHandle_t batteryTaskHandle = nullptr;
  static void batteryTask(void* pvParameters);
  constexpr TickType_t kTaskStopTimeoutTicks = pdMS_TO_TICKS(1000);
}

void battery::startBatteryTask() {
  // Always request a clean stop first so rapid stop/start sequences remain safe.
  stopBatteryTask();
  if (batteryTaskHandle != nullptr) {
    dbgError("Battery task did not stop cleanly");
    return;
  }

  BaseType_t rc = xTaskCreate(batteryTask,       // Task function
                              "Battery Monitor", // Task name
                              512,               // Stack size (words)
                              nullptr,           // Task parameters
                              0,                 // Priority
                              &batteryTaskHandle // Save handle
  );
  if (rc != pdPASS) {
    batteryTaskHandle = nullptr;
    dbgError("Failed to start battery task");
  }
}

void battery::stopBatteryTask() {
  if (!taskSync::requestStopAndWait(batteryTaskHandle, kTaskStopTimeoutTicks)) dbgWarn("Battery task stop timed out");
}

void battery::batteryTask(void* pvParameters) {
  while (true) {
    // Stop signal can be consumed immediately or while waiting for next period.
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;

    uint8_t batteryPercent = readLevel(kNumADCSamplesToAverage);
    bluetooth::blebas.notify(batteryPercent);

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBatteryUpdateRate)) > 0) break;
  }

  batteryTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
