/**
 * @file gatt.cpp
 * @brief Custom GATT service/characteristic setup and callback handlers.
 */

#include "bluetooth/gatt.h"
#include "HWConfig/constants.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/transmitdata_task.h"
#include "sensors/sensor_manager.h"
#include "storage/storage.h"
#include "util/debug_log.h"
#include "util/payload_validation.h"

#include <bluefruit.h>

namespace bluetooth {
  BLECharacteristic chrStatus(kUUIDChrStatus);
  BLECharacteristic chrDeviceName(kUUIDChrDeviceName);
  BLECharacteristic chrSensorCtrl(kUUIDChrSensorCtrl);
  BLECharacteristic chrSensorData(kUUIDChrSensorData);
  BLECharacteristic chrSensorParams(kUUIDChrSensorParams);

} // namespace bluetooth

void bluetooth::initGatt() {
  dbgInfo("Initializing custom BLE services...");

  bleService.begin();

  // Status characteristic
  chrStatus.setProperties(CHR_PROPS_READ);
  chrStatus.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrStatus.setFixedLen(1);
  chrStatus.begin();
  chrStatus.write8(0);

  // Device name characteristic
  chrDeviceName.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  chrDeviceName.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDeviceName.setMaxLen(BLE_GAP_DEVNAME_MAX_LEN);
  chrDeviceName.begin();
  chrDeviceName.write(storage::readDeviceName().c_str());
  chrDeviceName.setWriteCallback(onNameWrite);

  // Sensor control
  chrSensorCtrl.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrSensorCtrl.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrSensorCtrl.setFixedLen(1);
  chrSensorCtrl.begin();
  chrSensorCtrl.setWriteCallback(onSensorControl);

  // Sensor Parameters
  chrSensorParams.setProperties(CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_WRITE);
  chrSensorParams.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chrSensorParams.setMaxLen(kMTURequest - kATTHeaderLen);
  chrSensorParams.begin();
  chrSensorParams.setWriteCallback(onSensorParameters);

  // Sensor Data
  chrSensorData.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  chrSensorData.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chrSensorData.setMaxLen(kMTURequest - kATTHeaderLen); // We set this to the MAX, but we will only send in MTU chunks
  chrSensorData.begin();
}

void bluetooth::onNameWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  constexpr uint16_t kMaxNameLen = BLE_GAP_DEVNAME_MAX_LEN - 1;
  if (!payloadValidation::requireLengthInRange(data, len, 1, kMaxNameLen, "device name")) return;

  String name;
  name.reserve(len);
  for (uint16_t i = 0; i < len; ++i) {
    name += static_cast<char>(data[i]);
  }

  dbgInfo("Updating device name = " + name);
  storage::writeDeviceName(name);
}

void bluetooth::onSensorControl(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (!payloadValidation::requireLengthInRange(data, len, 1, 1, "EChem control")) return;

  dbgInfo("Received EChem Control Command");
  if (sensor::controlCommand(data, len)) clearTransmitBuffer();
}

void bluetooth::onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  constexpr uint16_t kMaxParamsLen = kMTURequest - kATTHeaderLen;
  if (!payloadValidation::requireLengthInRange(data, len, 1, kMaxParamsLen, "EChem parameters")) return;

  sensor::loadParameters(data, len);
}
