/**
 * @file storage.h
 * @brief Persistent storage API for device configuration data.
 */

#pragma once

#include <Arduino.h>

namespace storage {

  /**
   * @brief Initialize internal filesystem backend used for persistent settings.
   */
  void init();
  /**
   * @brief Persist a new BLE device name to internal storage.
   * @param name New device name string to store.
   */
  void writeDeviceName(const String& name);
  /**
   * @brief Load persisted BLE device name from storage (or default).
   * @return Device name string.
   */
  String readDeviceName();

} // namespace storage
