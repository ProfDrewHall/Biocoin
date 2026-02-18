/**
 * @file transmitdata_task.h
 * @brief BLE data transmit task lifecycle and enqueue API.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace bluetooth {

  /**
   * @brief Create BLE transmit worker task and backing stream buffer.
   */
  void createTransmitTask();
  /**
   * @brief Request BLE transmit task shutdown and wait for task exit.
   */
  void stopTransmitTask();
  /**
   * @brief Queue bytes for BLE notification and wake TX task.
   * @param data Serialized payload bytes to send over BLE notifications.
   */
  void startTransmitTask(const std::vector<uint8_t>& data);
  /**
   * @brief Drain and clear any pending bytes in BLE TX stream buffer.
   */
  void clearTransmitBuffer();

} // namespace bluetooth
