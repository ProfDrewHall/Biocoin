#pragma once

#include <cstdint>

namespace power {

  enum class PullConfig { Disabled, Pullup, Pulldown };

  void init();
  void initPins();
  void enableDCDC();
  void disconnectInputGPIO(uint32_t pin);
  void reconnectInputGPIO(uint32_t pin, PullConfig pull);

  void powerOnAFE(uint8_t muxChannel = 0);
  void powerOnTempSensor();
  void powerOnIontophoresis();
  void powerOffPeripherials();
} // namespace power
