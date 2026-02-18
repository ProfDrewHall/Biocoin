/**
 * @file sensor_manager.h
 * @brief Active-sensor selection, control, and status management API.
 */

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

  /**
   * @brief Initialize sensor manager state and supporting worker tasks.
   */
  void init();
  /**
   * @brief Get pointer to currently active sensor implementation.
   * @return Active sensor pointer, or null if none selected.
   */
  Sensor* getActiveSensor();
  /**
   * @brief Get currently selected sensor technique type.
   * @return Active sensor type enum.
   */
  SensorType getActiveSensorType();
  /**
   * @brief Get latest published sensor/test state.
   * @return Current test state enum.
   */
  TestState getTestState();
  /**
   * @brief Select technique and load parameter payload into active sensor.
   * @param data Payload bytes where first byte is `SensorType`.
   * @param len Payload length in bytes.
   * @return True when sensor exists and parameters are accepted.
   */
  bool loadParameters(uint8_t* data, uint16_t len);
  /**
   * @brief Execute sensor control command (start/stop).
   * @param data Command payload bytes.
   * @param len Payload length in bytes.
   * @return True when command executes successfully.
   */
  bool controlCommand(uint8_t* data, uint16_t len);
  /**
   * @brief ISR entry callback that schedules data-mover task.
   */
  void interruptHandler();
  /**
   * @brief Stop and release currently active sensor instance.
   */
  void cleanupSensor();
  /**
   * @brief Move buffered sensor data into BLE TX queue.
   * @param minBytesRequired Minimum bytes required before queueing (0 to flush all).
   */
  void queueDataForTX(size_t minBytesRequired);
  /**
   * @brief Update internal test state and publish to BLE status characteristic.
   * @param state New test state.
   */
  void updateStatus(TestState state);
} // namespace sensor
