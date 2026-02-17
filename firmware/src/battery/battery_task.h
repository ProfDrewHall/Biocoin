// Module: Battery task lifecycle API.
// Purpose: Start/stop the periodic battery monitor worker.

#pragma once

#include <Arduino.h>

namespace battery {
  /// Start the periodic battery monitor task (idempotent-safe).
  void startBatteryTask();
  /// Request the battery task to stop and wait for shutdown.
  void stopBatteryTask();
} // namespace battery
