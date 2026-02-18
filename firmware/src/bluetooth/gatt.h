#pragma once

#include "bluefruit.h"

namespace bluetooth {

  extern BLECharacteristic chrSensorData;
  extern BLECharacteristic chrStatus;


  void initGatt();
  void updateStatus();

  // Callbacks
  void onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len);
  void onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
  void onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

} // namespace bluetooth
