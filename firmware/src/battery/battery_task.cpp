#include "battery/battery_task.h"

#include "HWConfig/constants.h"
#include "battery/battery.h"
#include "bluetooth/bluetooth.h"
#include "util/debug_log.h"

#include <Arduino.h>

using namespace battery;

namespace battery {
  static TaskHandle_t batteryTaskHandle = nullptr;
  static void batteryTask(void* pvParameters);
}

void battery::startBatteryTask() {
  if (batteryTaskHandle == nullptr) {
    xTaskCreate(batteryTask,       // Task function
                "Battery Monitor", // Task name
                512,               // Stack size (words)
                nullptr,           // Task parameters
                0,                 // Priority
                &batteryTaskHandle // Save handle
    );
  }
}

void battery::stopBatteryTask() {
  if (batteryTaskHandle != nullptr) {
    vTaskDelete(batteryTaskHandle);
    batteryTaskHandle = nullptr;
  }
}

void battery::batteryTask(void* pvParameters) {
  while (true) {
    uint8_t batteryPercent = readLevel(kNumADCSamplesToAverage);
    bluetooth::blebas.write(batteryPercent);                        // Note: This is just a write, no power penalty for updating
    vTaskDelay(pdMS_TO_TICKS(kBatteryUpdateRate));
  }
}
