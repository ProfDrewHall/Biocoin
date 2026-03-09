/**
 * @file battery.h
 * @brief Battery monitoring service public API.
 */

#pragma once

#include <stdint.h>

namespace battery {
  /**
   * @brief Initialize battery-monitoring subsystem state.
   */
  void init(void);

  /**
   * @brief Start periodic battery measurements.
   */
  void start(void);
  /**
   * @brief Stop periodic battery measurements.
   */
  void stop(void);

  /**
   * @brief Read battery voltage using averaged ADC measurements.
   * @param numToAverage Number of ADC readings to average.
   * @param disconnected Optional output set true when the battery input appears disconnected/floating.
   * @return Battery voltage in volts.
   */
  float readBatteryVoltageV(uint8_t numToAverage = 100, bool* disconnected = nullptr);
  /**
   * @brief Read battery percentage using averaged ADC measurements.
   * @param numToAverage Number of ADC readings to average.
   * @return Battery level as percentage [0..100].
   */
  uint8_t readBatteryPercent(uint8_t numToAverage = 100);
  /**
   * @brief Convert measured battery voltage into percent full using fitted sigmoid curve.
   * @param voltage Battery voltage in volts.
   * @return Battery level as percentage [0..100].
   */
  uint8_t voltageToPercent(float voltage);
} // namespace battery
