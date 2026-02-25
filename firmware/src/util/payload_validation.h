/**
 * @file payload_validation.h
 * @brief Reusable payload length/null validation helpers for BLE handlers.
 * @details Centralizes defensive payload checks so BLE write handlers fail consistently and log context-rich warnings.
 */

#pragma once

#include "util/debug_log.h"

#include <cstdint>
#include <cstring>
#include <type_traits>

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

  template <typename T>
  inline bool parseExactPayload(const uint8_t* data, uint16_t len, const char* context, T* out) {
    static_assert(std::is_trivially_copyable<T>::value, "Payload type must be trivially copyable");
    if (out == nullptr) return false;
    if (data == nullptr || len != sizeof(T)) {
      dbgWarn(String("Ignoring invalid payload length for ") + context + ": " + String(len) + String(" expected ") +
              String(sizeof(T)));
      return false;
    }
    memcpy(out, data, sizeof(T));
    return true;
  }
} // namespace payloadValidation
