#include "HWConfig/constants.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/gatt.h" // for kMaxTechniqueLen -- move?
#include "bluetooth/transmitdata_task.h"
#include "util/debug_log.h"

#include <Arduino.h>
#include <queue>

using namespace bluetooth;

namespace bluetooth {
  std::queue<uint8_t> TX_queue;
  static TaskHandle_t TXTaskHandle = nullptr;
  static void transmitTask(void* pvParameters);
} // namespace bluetooth

void bluetooth::createTransmitTask() {
  xTaskCreate(transmitTask,   // Task function
              "BLE TX",       // Task name
              2048,           // Stack size (in words)
              nullptr,        // Task parameters
              1,              // Priority (very low)
              &TXTaskHandle); // Task handle
}

void bluetooth::startTransmitTask(const std::vector<uint8_t>& data) {
  if (TXTaskHandle != nullptr) {

    // Move the data to our queue
    for (uint8_t byte : data)
      TX_queue.push(byte);

    xTaskNotifyGive(TXTaskHandle);
    vTaskResume(TXTaskHandle);
  }
}

// This task is woken up if there is data to be sent. It then grabs data from the TX_queue in frames and pushes it out
// over BLE. This was a concious design decision rather than storing the data indefinitely.
void bluetooth::transmitTask(void* pvParameters) {

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (!TX_queue.empty()) {
      uint8_t num_bytes = std::min((size_t)dataSize, TX_queue.size());
      uint8_t* data = new uint8_t[num_bytes];  // dynamically allocate

      for (int i = 0; i < num_bytes; ++i) {
        data[i] = TX_queue.front();
        TX_queue.pop();
      }

      TickType_t start = xTaskGetTickCount();
      const TickType_t timeout = pdMS_TO_TICKS(500);

      while (num_bytes > 0 && !chrSensorData.notify(data, num_bytes)) {
        if ((xTaskGetTickCount() - start) > timeout) {
          dbgWarn("Notify retry timed out.");
          break;
        }
        dbgError("Notify failed — retrying...");
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      dbgInfo("Sent " + String(num_bytes) + " Bytes");
      delete data;
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    dbgInfo("BLE transmit task suspending.");
    vTaskSuspend(nullptr);
  }
}
