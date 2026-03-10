/**
 * @file gatt.cpp
 * @brief Custom GATT service/characteristic setup and callback handlers.
 */

#include "bluetooth/gatt.h"
#include "HWConfig/constants.h"
#include "battery/battery.h"
#include "bluetooth/bluetooth.h"
#include "bluetooth/transmitdata_task.h"
#include "digital/digital_io_manager.h"
#include "power/power.h"
#include "sensors/core/sensor_manager.h"
#include "storage/storage.h"
#include "util/debug_log.h"
#include "util/payload_validation.h"

#include <bluefruit.h>

namespace bluetooth {
  BLECharacteristic chrStatus(kUUIDChrStatus);
  BLECharacteristic chrDeviceName(kUUIDChrDeviceName);
  BLECharacteristic chrDigitalConfig(kUUIDChrDigitalConfig);
  BLECharacteristic chrDigitalValue(kUUIDChrDigitalValue);
#if BIOCOIN_ENABLE_DEBUG_GATT
  BLECharacteristic chrDebugBatteryMillivolts(kUUIDChrDebugBattery);
  BLECharacteristic chrDebugAFEPower(kUUIDChrDebugAFEPower);
#endif
  BLECharacteristic chrSensorCtrl(kUUIDChrSensorCtrl);
  BLECharacteristic chrSensorData(kUUIDChrSensorData);
  BLECharacteristic chrSensorParams(kUUIDChrSensorParams);

} // namespace bluetooth

namespace {
  void replyWithReadPayload(uint16_t conn_hdl, ble_gatts_evt_read_t* request, const std::vector<uint8_t>& payload) {
    ble_gatts_rw_authorize_reply_params_t reply = {};
    reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
    reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
    reply.params.read.update = 1;
    reply.params.read.offset = 0;
    reply.params.read.len = 0;
    reply.params.read.p_data = nullptr;

    if (request == nullptr) {
      reply.params.read.gatt_status = BLE_GATT_STATUS_ATTERR_UNLIKELY_ERROR;
      sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
      return;
    }

    if (request->offset > payload.size()) {
      reply.params.read.gatt_status = BLE_GATT_STATUS_ATTERR_INVALID_OFFSET;
      sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
      return;
    }

    BLEConnection* conn = Bluefruit.Connection(conn_hdl);
    const uint16_t mtuPayload =
        (conn != nullptr && conn->getMtu() > bluetooth::kATTHeaderLen)
            ? (conn->getMtu() - bluetooth::kATTHeaderLen)
            : (23u - bluetooth::kATTHeaderLen);
    const size_t available = payload.size() - request->offset;
    const uint16_t readLen = static_cast<uint16_t>(min(static_cast<size_t>(mtuPayload), available));

    reply.params.read.offset = request->offset;
    reply.params.read.len = readLen;
    reply.params.read.p_data = const_cast<uint8_t*>(payload.data()) + request->offset;
    sd_ble_gatts_rw_authorize_reply(conn_hdl, &reply);
  }
} // namespace

#if BIOCOIN_ENABLE_DEBUG_GATT
namespace {
  constexpr uint8_t kDebugBatteryCommandSampleNow = 0x01;
  bool gDebugAFEPowerEnabled = false;

  uint16_t sampleDebugBatteryVoltageMillivolts() {
    const float voltageV = battery::readBatteryVoltageV(battery::kNumADCSamplesToAverage);
    const float millivolts = voltageV * 1000.0f;
    if (millivolts <= 0.0f) return 0;
    if (millivolts >= 65535.0f) return 65535;
    return static_cast<uint16_t>(millivolts + 0.5f);
  }

  void writeDebugAFEState(bool enabled) {
    const uint8_t state = enabled ? 1u : 0u;
    bluetooth::chrDebugAFEPower.write8(state);
    bluetooth::chrDebugAFEPower.notify8(state);
  }
} // namespace
#endif

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

#if BIOCOIN_ENABLE_DEBUG_GATT
  // Debug Battery Voltage (mV). Write command 0x01 forces an immediate ADC refresh.
  chrDebugBatteryMillivolts.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  chrDebugBatteryMillivolts.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDebugBatteryMillivolts.setMaxLen(2);
  chrDebugBatteryMillivolts.begin();
  chrDebugBatteryMillivolts.write16(sampleDebugBatteryVoltageMillivolts());
  chrDebugBatteryMillivolts.setWriteCallback(onDebugBatteryCommand);

  // Debug AFE burn-load control. 0 = off, non-zero = on.
  chrDebugAFEPower.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  chrDebugAFEPower.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDebugAFEPower.setFixedLen(1);
  chrDebugAFEPower.begin();
  chrDebugAFEPower.write8(0);
  chrDebugAFEPower.setWriteCallback(onDebugAFEPowerWrite);
#endif

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

  // Digital Config
  chrDigitalConfig.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrDigitalConfig.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDigitalConfig.setMaxLen(digital::getConfigPayloadSize());
  chrDigitalConfig.begin();
  const std::vector<uint8_t> initialDigitalConfig = digital::readConfig();
  chrDigitalConfig.write(initialDigitalConfig.data(), initialDigitalConfig.size());
  chrDigitalConfig.setWriteCallback(onDigitalConfigWrite);
  chrDigitalConfig.setReadAuthorizeCallback(onDigitalConfigRead);

  // Digital Value
  chrDigitalValue.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chrDigitalValue.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  chrDigitalValue.setMaxLen(digital::getValuePayloadSize());
  chrDigitalValue.begin();
  const std::vector<uint8_t> initialDigitalValue = digital::readValue();
  chrDigitalValue.write(initialDigitalValue.data(), initialDigitalValue.size());
  chrDigitalValue.setWriteCallback(onDigitalValueWrite);
  chrDigitalValue.setReadAuthorizeCallback(onDigitalValueRead);

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
  const sensor::SensorCmd cmd = static_cast<sensor::SensorCmd>(data[0]);
  if (cmd == sensor::SensorCmd::START) {
    // Drop stale bytes from a prior run before starting a new one.
    clearTransmitBuffer();
  }

  sensor::controlCommand(data, len);
}

void bluetooth::onSensorParameters(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  constexpr uint16_t kMaxParamsLen = kMTURequest - kATTHeaderLen;
  if (!payloadValidation::requireLengthInRange(data, len, 1, kMaxParamsLen, "EChem parameters")) return;

  sensor::loadParameters(data, len);
}

void bluetooth::onDigitalConfigWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  const uint16_t kMaxLen = static_cast<uint16_t>(digital::getConfigPayloadSize());
  if (!payloadValidation::requireLengthInRange(data, len, sizeof(digital::DigitalConfigHeader), kMaxLen,
                                               "digital config"))
    return;

  if (!digital::writeConfig(data, len)) return;

  const std::vector<uint8_t> payload = digital::readConfig();
  chrDigitalConfig.write(payload.data(), payload.size());
}

void bluetooth::onDigitalValueWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  const uint16_t kMaxLen = static_cast<uint16_t>(digital::getValuePayloadSize());
  if (!payloadValidation::requireLengthInRange(data, len, sizeof(digital::DigitalValueHeader), kMaxLen,
                                               "digital value"))
    return;

  if (!digital::writeValue(data, len)) return;

  const std::vector<uint8_t> payload = digital::readValue();
  chrDigitalValue.write(payload.data(), payload.size());
}

void bluetooth::onDigitalConfigRead(uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t* request) {
  const std::vector<uint8_t> payload = digital::readConfig();
  chr->write(payload.data(), payload.size());
  replyWithReadPayload(conn_hdl, request, payload);
}

void bluetooth::onDigitalValueRead(uint16_t conn_hdl, BLECharacteristic* chr, ble_gatts_evt_read_t* request) {
  const std::vector<uint8_t> payload = digital::readValue();
  chr->write(payload.data(), payload.size());
  replyWithReadPayload(conn_hdl, request, payload);
}

#if BIOCOIN_ENABLE_DEBUG_GATT
void bluetooth::updateDebugBatteryVoltageMillivolts(uint16_t millivolts) {
  chrDebugBatteryMillivolts.write16(millivolts);
  const uint8_t payload[2] = {static_cast<uint8_t>(millivolts & 0xffu), static_cast<uint8_t>(millivolts >> 8)};
  chrDebugBatteryMillivolts.notify(payload, sizeof(payload));
}

void bluetooth::onDebugBatteryCommand(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  if (!payloadValidation::requireLengthInRange(data, len, 1, 1, "debug battery command")) return;

  if (data[0] != kDebugBatteryCommandSampleNow) {
    dbgWarn(String("Unknown debug battery command: ") + data[0]);
    return;
  }

  const uint16_t voltageMillivolts = sampleDebugBatteryVoltageMillivolts();
  updateDebugBatteryVoltageMillivolts(voltageMillivolts);
}

void bluetooth::onDebugAFEPowerWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) {
  if (!payloadValidation::requireLengthInRange(data, len, 1, 1, "debug AFE power command")) return;

  if (sensor::getTestState() == sensor::TestState::RUNNING) {
    dbgWarn("Ignoring debug AFE command while a technique is running");
    writeDebugAFEState(gDebugAFEPowerEnabled);
    return;
  }

  const bool requestedEnabled = (data[0] != 0u);
  if (requestedEnabled != gDebugAFEPowerEnabled) {
    if (requestedEnabled) {
      power::powerOnAFE(0);
      dbgInfo("Debug AFE burn load enabled");
    } else {
      power::powerOffPeripherals();
      dbgInfo("Debug AFE burn load disabled");
    }
    gDebugAFEPowerEnabled = requestedEnabled;
  }

  writeDebugAFEState(gDebugAFEPowerEnabled);
}

void bluetooth::resetDebugStateOnDisconnect() {
  if (gDebugAFEPowerEnabled) {
    power::powerOffPeripherals();
    gDebugAFEPowerEnabled = false;
    dbgInfo("Debug AFE burn load disabled on disconnect");
  }
  chrDebugAFEPower.write8(0);
}
#endif
