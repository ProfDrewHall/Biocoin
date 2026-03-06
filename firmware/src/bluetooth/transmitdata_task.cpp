/**
 * @file transmitdata_task.cpp
 * @brief BLE notify transmit worker backed by a bounded stream buffer.
 */

#include "bluetooth/bluetooth.h"
#include "bluetooth/gatt.h"
#include "bluetooth/transmitdata_task.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

#include <Arduino.h>
#include <algorithm>
#include <atomic>
#include <stream_buffer.h>
#include <vector>

namespace bluetooth {
  static StreamBufferHandle_t TXStream = nullptr;
  static TaskHandle_t TXTaskHandle = nullptr;
  static std::atomic_bool TXTaskStopRequested{false};
  static void transmitTask(void* pvParameters);

  constexpr size_t kTxStreamCapacity = 8192;
  constexpr TickType_t kTxEnqueueTimeoutTicks = pdMS_TO_TICKS(20);
  constexpr size_t kTxChunkBufferBytes = 256;
  constexpr TickType_t kTxTaskStopTimeoutTicks = pdMS_TO_TICKS(1000);
} // namespace bluetooth

void bluetooth::createTransmitTask() {
  if (TXTaskHandle != nullptr) return;
  TXTaskStopRequested.store(false, std::memory_order_relaxed);

  if (TXStream == nullptr) {
    TXStream = xStreamBufferCreate(kTxStreamCapacity, 1);
    if (TXStream == nullptr) {
      dbgError("Failed to create BLE TX stream buffer");
      return;
    }
  }

  BaseType_t rc = xTaskCreate(transmitTask,   // Task function
                              "BLE TX",       // Task name
                              2048,           // Stack size (in words)
                              nullptr,        // Task parameters
                              1,              // Priority (very low)
                              &TXTaskHandle); // Task handle
  if (rc != pdPASS) {
    TXTaskHandle = nullptr;
    dbgError("Failed to start BLE TX task");
  }
}

void bluetooth::stopTransmitTask() {
  if (TXTaskHandle == nullptr) return;

  TXTaskStopRequested.store(true, std::memory_order_relaxed);
  if (!taskSync::requestStopAndWait(TXTaskHandle, kTxTaskStopTimeoutTicks)) dbgWarn("BLE TX task stop timed out");
}

void bluetooth::startTransmitTask(const std::vector<uint8_t>& data) {
  if (TXTaskHandle == nullptr || TXStream == nullptr || data.empty()) return;

  size_t bytesSent = xStreamBufferSend(TXStream, data.data(), data.size(), kTxEnqueueTimeoutTicks);
  if (bytesSent < data.size()) {
    dbgWarn("BLE TX buffer full, dropped " + String(data.size() - bytesSent) + " bytes");
  }

  xTaskNotifyGive(TXTaskHandle);
}

void bluetooth::clearTransmitBuffer() {
  if (TXStream == nullptr) return;

  uint8_t sink[kTxChunkBufferBytes];
  while (xStreamBufferReceive(TXStream, sink, sizeof(sink), 0) > 0) {
  }
}

// This task is woken up if there is data to be sent. It then grabs data from the stream in frames and pushes it out
// over BLE. This was a conscious design decision rather than storing the data indefinitely.
void bluetooth::transmitTask(void* pvParameters) {
  uint8_t txChunk[kTxChunkBufferBytes];

  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (TXTaskStopRequested.load(std::memory_order_relaxed)) break;

    size_t maxChunk = std::min(sizeof(txChunk), static_cast<size_t>(dataSize));
    size_t numBytes = xStreamBufferReceive(TXStream, txChunk, maxChunk, 0);
    while (numBytes > 0 && !TXTaskStopRequested.load(std::memory_order_relaxed)) {
      uint16_t notifyLen = static_cast<uint16_t>(numBytes);

      TickType_t start = xTaskGetTickCount();
      const TickType_t timeout = pdMS_TO_TICKS(500);

      bool notifyOk = false;
      while (notifyLen > 0 && !TXTaskStopRequested.load(std::memory_order_relaxed)) {
        if (chrSensorData.notify(txChunk, notifyLen)) {
          notifyOk = true;
          break;
        }
        if ((xTaskGetTickCount() - start) > timeout) {
          dbgWarn("Notify retry timed out.");
          break;
        }
        dbgWarn("Notify failed - retrying...");
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (notifyOk) {
        dbgInfo("Sent " + String(notifyLen) + " bytes");
        vTaskDelay(pdMS_TO_TICKS(50));
      } else if (!TXTaskStopRequested.load(std::memory_order_relaxed)) {
        dbgWarn("Dropped " + String(notifyLen) + " bytes after notify timeout");
      }

      numBytes = xStreamBufferReceive(TXStream, txChunk, maxChunk, 0);
    }

    if (TXTaskStopRequested.load(std::memory_order_relaxed)) break;
  }

  TXTaskStopRequested.store(false, std::memory_order_relaxed);
  TXTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
