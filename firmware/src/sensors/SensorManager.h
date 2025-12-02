#pragma once

#include "sensors/sensor.h"
#include <memory>

namespace sensor {

  enum class SensorType : uint8_t {
    None = 0x00,
    CA = 0x01,
    CV = 0x02,
    DPV = 0x03,
    IMP = 0x04,
    OCP = 0x05,
    SWV = 0x06,
    //EIS = 0x06,
    TEMP = 0x10,
    IONTOPHORESIS = 0x20
  };

  enum class SensorCmd : uint8_t {
    START = 0x01,
    STOP = 0xFF,
  };

  enum class TestState : uint8_t { NOT_RUNNING = 0x00, INVALID_PARAMETERS, RUNNING, ERROR, CURRENT_LIMIT_EXCEEDED };

  extern std::unique_ptr<Sensor> pActiveSensor;
  extern SensorType activeSensorID;
  //extern TestState testState;

  void init();
  bool loadParameters(uint8_t* data, uint16_t len);
  bool controlCommand(uint8_t* data, uint16_t len);
  void interruptHandler();
  void cleanupSensor();
  void queueDataForTX(size_t minBytesRequired);
  void updateStatus(TestState state);
} // namespace sensor
