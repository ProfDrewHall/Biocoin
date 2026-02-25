/**
 * @file parameter_io_helpers.h
 * @brief Shared payload parsing and parameter logging helpers for techniques.
 */

#pragma once

#include "util/debug_log.h"
#include "util/payload_validation.h"

#include <cstdint>

namespace sensor::parameterIO {
  template <typename PayloadT>
  inline bool parsePayload(const char* techniqueName, const uint8_t* data, uint16_t len, PayloadT* out) {
    return payloadValidation::parseExactPayload(data, len, techniqueName, out);
  }

  template <typename ValueT>
  inline void logField(const char* labelWithUnitAndColon, const ValueT& value) {
    dbgInfo(String("\t") + String(labelWithUnitAndColon) + String(value));
  }

  inline void logBoolField(const char* labelWithColon, bool value, const char* trueText, const char* falseText) {
    dbgInfo(String("\t") + String(labelWithColon) + String(value ? trueText : falseText));
  }
} // namespace sensor::parameterIO
