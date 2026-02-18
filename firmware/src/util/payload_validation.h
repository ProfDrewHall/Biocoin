// Module: Payload validation helpers.
// Purpose: Centralize common null/length checks for BLE payload handlers.

#pragma once

#include "util/debug_log.h"

#include <cstdint>

namespace payloadValidation {
  inline bool requireMinLength(const uint8_t* data, uint16_t len, uint16_t minLen, const char* context) {
    if (data == nullptr || len < minLen) {
      dbgWarn(String("Ignoring invalid payload for ") + context);
      return false;
    }
    return true;
  }

  inline bool requireLengthInRange(const uint8_t* data, uint16_t len, uint16_t minLen, uint16_t maxLen,
                                   const char* context) {
    if (data == nullptr || len < minLen || len > maxLen) {
      dbgWarn(String("Ignoring invalid payload length for ") + context + ": " + String(len));
      return false;
    }
    return true;
  }
} // namespace payloadValidation
