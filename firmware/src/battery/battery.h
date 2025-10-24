#pragma once

#include <stdint.h>

namespace battery {
  void init(void);
  
  void start(void);
  void stop(void);
  
  uint8_t readLevel(uint8_t numToAverage = 100);
  uint8_t mapBatteryLevel(float voltage);
} // namespace battery