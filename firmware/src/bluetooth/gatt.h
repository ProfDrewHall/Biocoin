/**
 * @file gatt.h
 * @brief Custom GATT characteristic declarations and write callbacks.
 */

#pragma once

#include "bluefruit.h"

namespace bluetooth {

  extern BLECharacteristic chrSensorData;
  extern BLECharacteristic chrStatus;


  /**
   * @brief Initialize custom GATT service and characteristics.
   */
  void initGatt();
  /**
   * @brief Update status characteristic value for host consumption.
   */
  void updateStatus();

  // Callbacks
  /**
   * @brief Handle writes to device-name characteristic and persist updated value.
   * @param Connection handle (unused in handler).
   * @param Characteristic pointer (unused in handler).
   * @param data Raw payload bytes.
   * @param len Payload length in bytes.
   */
  void onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);
  /**
   * @brief Handle start/stop control command writes.
   * @param conn_hdl Active connection handle.
   * @param chr Source characteristic.
   * @param data Control payload bytes.
   * @param len Payload length in bytes.
   */
  void onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  /**
   * @brief Handle sensor-parameter payload writes from host.
   * @param conn_hdl Active connection handle.
   * @param chr Source characteristic.
   * @param data Parameter payload bytes.
   * @param len Payload length in bytes.
   */
  void onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

} // namespace bluetooth
