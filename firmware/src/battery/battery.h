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
   * @brief Read battery percentage using averaged ADC measurements.
   * @param numToAverage Number of ADC readings to average.
   * @return Battery level as percentage [0..100].
   */
  uint8_t readLevel(uint8_t numToAverage = 100);
  /**
   * @brief Convert measured battery voltage into percent full.
   * @param voltage Battery voltage in volts.
   * @return Battery level as percentage [0..100].
   */
  uint8_t mapBatteryLevel(float voltage);
} // namespace battery
