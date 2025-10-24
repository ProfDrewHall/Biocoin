#pragma once

#include "bluefruit.h"
#include <queue>

namespace bluetooth {

  extern BLECharacteristic chrSensorData;
  extern BLECharacteristic chrStatus;


  void initGatt();

  template <typename T>
  void clearQueue(std::queue<T>& q);
  void updateStatus();

  // Callbacks
  void onDataNotifications(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
  void onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);
  void onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  void onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

} // namespace bluetooth
