/**
 * @file battery_task.h
 * @brief Battery monitor task lifecycle API.
 * @details Declares start/stop control functions for the periodic battery monitor worker task.
 */

#pragma once

#include <Arduino.h>

namespace battery {
  /**
   * @brief Start the periodic battery monitor task.
   * @details Safe to call repeatedly; implementation handles stop/start sequencing.
   */
  void startBatteryTask();
  /**
   * @brief Request battery monitor task shutdown and wait for completion.
   */
  void stopBatteryTask();
} // namespace battery
