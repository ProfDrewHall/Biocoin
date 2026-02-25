/**
 * @file sensor_validation.h
 * @brief Shared validation helpers for sensor payload parsing.
 */

#pragma once

#include <Arduino.h>

#include <cmath>
#include <initializer_list>

namespace sensor::validation {
  inline bool requireFinite(std::initializer_list<float> values, const String& reasonMessage, String* reason) {
    for (const float value : values) {
      if (!std::isfinite(value)) {
        if (reason) *reason = reasonMessage;
        return false;
      }
    }
    return true;
  }

  inline bool requirePositive(float value, const String& reasonMessage, String* reason) {
    if (value <= 0.0f) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireNonNegative(float value, const String& reasonMessage, String* reason) {
    if (value < 0.0f) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireNonZero(float value, const String& reasonMessage, String* reason) {
    if (value == 0.0f) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireGreaterOrEqual(float value, float minimum, const String& reasonMessage, String* reason) {
    if (value < minimum) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireLessOrEqual(float value, float maximum, const String& reasonMessage, String* reason) {
    if (value > maximum) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireBinaryFlag(uint8_t value, const String& reasonMessage, String* reason) {
    if (value > 1u) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireChannelInRange(uint8_t channel, uint8_t maxChannel, const String& reasonMessage, String* reason) {
    if (channel > maxChannel) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  inline bool requireWithinRange(float value, float minValue, float maxValue, const String& reasonMessage,
                                 String* reason) {
    if (value < minValue || value > maxValue) {
      if (reason) *reason = reasonMessage;
      return false;
    }
    return true;
  }

  template <typename T>
  inline bool requireOneOf(T value, std::initializer_list<T> allowed, const String& reasonMessage, String* reason) {
    for (const T allowedValue : allowed) {
      if (value == allowedValue) return true;
    }
    if (reason) *reason = reasonMessage;
    return false;
  }
} // namespace sensor::validation
