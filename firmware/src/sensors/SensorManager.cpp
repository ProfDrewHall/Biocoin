#include "sensors/SensorManager.h"

#include "bluetooth/gatt.h"
#include "bluetooth/transmitdata_task.h"
#include "drivers/ad5940_hal.h"
#include "sensors/EChem_CA.h"
#include "sensors/EChem_CV.h"
#include "sensors/EChem_DPV.h"
#include "sensors/EChem_SWV.h"
#include "sensors/EChem_Imp.h"
#include "sensors/EChem_OCP.h"
#include "sensors/EChem_Temp.h"
#include "sensors/datamover_task.h"
#include "sensors/Iontophoresis.h"
#include "util/debug_log.h"
#include "util/payload_validation.h"

#include <utility>
#include <memory>

namespace sensor {
  static std::unique_ptr<Sensor> pActiveSensor = nullptr;
  static SensorType activeSensorID = SensorType::None;
  static TestState testState = TestState::NOT_RUNNING;

  static std::unique_ptr<Sensor> createSensor(sensor::SensorType type);
}; // namespace sensor

void sensor::init() {
  dbgInfo("Initializing sensor interface...");
  createDataMoverTask(); // Create the task to handle interrupts
  updateStatus(TestState::NOT_RUNNING);
}

sensor::Sensor* sensor::getActiveSensor() {
  return pActiveSensor.get();
}

sensor::SensorType sensor::getActiveSensorType() {
  return activeSensorID;
}

sensor::TestState sensor::getTestState() {
  return testState;
}

std::unique_ptr<sensor::Sensor> sensor::createSensor(sensor::SensorType type) {
  dbgInfo("Creating sensor of type: " + String(static_cast<uint8_t>(type)));
  switch (type) {
  case SensorType::CA:
    return std::unique_ptr<Sensor>(new sensor::EChem_CA());
  case SensorType::CV:
    return std::unique_ptr<Sensor>(new sensor::EChem_CV());
  case SensorType::DPV:
    return std::unique_ptr<Sensor>(new sensor::EChem_DPV());
  case SensorType::SWV:
    return std::unique_ptr<Sensor>(new sensor::EChem_SWV());
  case SensorType::IMP:
    return std::unique_ptr<Sensor>(new sensor::EChem_Imp());
  case SensorType::OCP:
    return std::unique_ptr<Sensor>(new sensor::EChem_OCP());
  case SensorType::TEMP:
    return std::unique_ptr<Sensor>(new sensor::EChem_Temp());
  case SensorType::IONTOPHORESIS:
    return std::unique_ptr<Sensor>(new sensor::Iontophoresis());
  case SensorType::None:
  default:
    return nullptr;
  }
}

bool sensor::loadParameters(uint8_t* data, uint16_t len) {
  if (!payloadValidation::requireMinLength(data, len, 1, "sensor parameters")) {
    updateStatus(TestState::INVALID_PARAMETERS);
    return false;
  }

  const SensorType requestedType = static_cast<SensorType>(data[0]);

  if (!pActiveSensor || requestedType != activeSensorID) {
    cleanupSensor();

    std::unique_ptr<Sensor> next = createSensor(requestedType);
    if (!next) {
      dbgError(String("Unknown or unsupported sensor: ") + data[0]);
      updateStatus(TestState::INVALID_PARAMETERS);
      return false;
    }

    pActiveSensor = std::move(next);
    activeSensorID = requestedType;
    updateStatus(TestState::NOT_RUNNING);
  }

  // Forward parameters to the newly created technique
  if (!pActiveSensor->loadParameters(data + 1, len - 1)) {
    updateStatus(TestState::INVALID_PARAMETERS);
    return false;
  }

  return true;
}

bool sensor::controlCommand(uint8_t* data, uint16_t len) {
  if (!payloadValidation::requireMinLength(data, len, 1, "sensor control") || pActiveSensor == nullptr) {
    updateStatus(TestState::INVALID_PARAMETERS);
    return false;
  }

  SensorCmd cmd = static_cast<SensorCmd>(data[0]);
  switch (cmd) {
  case SensorCmd::START:
    if (!pActiveSensor->start()) {
      updateStatus(TestState::ERROR);
      return false;
    }
    if (activeSensorID != SensorType::IONTOPHORESIS) enableAFEInterrupt(interruptHandler);

    updateStatus(TestState::RUNNING);
    break;
  case SensorCmd::STOP:
    if (!pActiveSensor->stop()) {
      updateStatus(TestState::ERROR);
      return false;
    }
    disableAFEInterrupt();
    queueDataForTX(0); // Push out any remaining data in the queue
    updateStatus(TestState::NOT_RUNNING);
    break;
  default: // Unknown command
    dbgError(String("Unknown control command: ") + data[0]);
    updateStatus(TestState::INVALID_PARAMETERS);
    return false;
  }

  return true;
}

void sensor::interruptHandler() {
  startDataMoverTask(); // Deal with the interrupt
}

void sensor::cleanupSensor() {
  disableAFEInterrupt(); // Stop interrupts
  if (pActiveSensor)
    pActiveSensor = nullptr;

  updateStatus(TestState::NOT_RUNNING);
}

void sensor::queueDataForTX(size_t minBytesRequired) {
  if (pActiveSensor == nullptr) return;

  size_t available = pActiveSensor->getNumBytesAvailable();
  if (available == 0) return;

  if (minBytesRequired > 0 && available < minBytesRequired) // Is there enough data to send?
    return;

  size_t bytesToSend = (minBytesRequired == 0) ? available : minBytesRequired;
  std::vector<uint8_t> data = pActiveSensor->getData(bytesToSend);

  dbgInfo("Queued " + String(bytesToSend) + " bytes for TX...");
  bluetooth::startTransmitTask(data);
}

void sensor::updateStatus(sensor::TestState state) {
  testState = state;
  bluetooth::chrStatus.write8(static_cast<uint8_t>(testState));
}
