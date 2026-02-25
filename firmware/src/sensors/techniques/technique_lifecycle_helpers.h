/**
 * @file technique_lifecycle_helpers.h
 * @brief Shared lifecycle helpers for AD5940 periodic sensor techniques.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "util/debug_log.h"

#include <Arduino.h>
#include <cmath>

namespace sensor::techniqueLifecycle {
  inline uint32_t deriveFifoThreshold(float processingIntervalS, float samplingIntervalS, uint32_t minThreshold,
                                      uint32_t maxThreshold, uint32_t granularity = 1u, float multiplier = 1.0f) {
    if (granularity == 0u) granularity = 1u;

    const float thresholdRatio = multiplier * processingIntervalS / samplingIntervalS;
    uint32_t threshold = static_cast<uint32_t>(std::lround(thresholdRatio));
    if (threshold < minThreshold) threshold = minThreshold;
    if (threshold > maxThreshold) threshold = maxThreshold;

    if (granularity > 1u) {
      threshold = (threshold / granularity) * granularity;
      if (threshold < minThreshold) threshold = minThreshold;
    }
    return threshold;
  }

  template <typename OnStop, typename OnStopped>
  inline void stopIfEndSequence(bool reachedEndSequence, const char* techniqueName, OnStop onStop, OnStopped onStopped) {
    if (!reachedEndSequence) return;

    dbgError(String(techniqueName) + String(" ENDSEQ interrupt received; stopping technique."));
    onStopped(onStop());
  }

  inline bool computeWakeupTimeTicks(float ticksFloat, uint32_t adjustTicks, const char* invalidReason, String* reason,
                                     uint32_t* outWakeupTicks) {
    if (outWakeupTicks == nullptr) return false;
    if (!std::isfinite(ticksFloat) || ticksFloat <= static_cast<float>(adjustTicks)) {
      if (reason) *reason = invalidReason;
      return false;
    }

    const uint32_t ticks = static_cast<uint32_t>(std::lround(ticksFloat));
    if (ticks <= adjustTicks) {
      if (reason) *reason = invalidReason;
      return false;
    }

    *outWakeupTicks = ticks - adjustTicks;
    return true;
  }

  inline bool computeWakeupTimeTicksFromSeconds(float lfoscFreq, float durationSeconds, uint32_t adjustTicks,
                                                const char* invalidReason, String* reason, uint32_t* outWakeupTicks) {
    const float ticksFloat = lfoscFreq * durationSeconds;
    return computeWakeupTimeTicks(ticksFloat, adjustTicks, invalidReason, reason, outWakeupTicks);
  }

  inline bool computeWakeupTimeTicksFromMs(float lfoscFreq, float durationMs, uint32_t adjustTicks,
                                           const char* invalidReason, String* reason, uint32_t* outWakeupTicks) {
    const float ticksFloat = lfoscFreq * durationMs / 1000.0f;
    return computeWakeupTimeTicks(ticksFloat, adjustTicks, invalidReason, reason, outWakeupTicks);
  }
} // namespace sensor::techniqueLifecycle
