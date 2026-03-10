/**
 * @file digital_pin_map.h
 * @brief Compile-time logical-channel mapping for the runtime digital I/O interface.
 */

#pragma once

#include "HWConfig/constants.h"

#include <cstddef>
#include <cstdint>

namespace digital {

enum class DigitalChannel : uint8_t {
  kAux0 = 0,
  kAux1 = 1,
  kAux2 = 2,
  kAux3 = 3,
};

enum class DigitalMode : uint8_t {
  kInput = 0,
  kOutput = 1,
  kFloating = 2,
  kPWM = 3,
};

struct DigitalPinInfo {
  DigitalChannel channel;
  uint32_t gpioPin;
  DigitalMode defaultMode;
  const char* name;
};

constexpr DigitalPinInfo kDigitalPins[] = {
    {DigitalChannel::kAux0, 23u, DigitalMode::kFloating, "P0.11"},
    {DigitalChannel::kAux1, 22u, DigitalMode::kFloating, "P0.12"},
    {DigitalChannel::kAux2, 1u, DigitalMode::kFloating, "P0.24"},
    {DigitalChannel::kAux3, 0u, DigitalMode::kFloating, "P0.25"},
};

constexpr size_t kNumDigitalPins = sizeof(kDigitalPins) / sizeof(kDigitalPins[0]);

} // namespace digital
