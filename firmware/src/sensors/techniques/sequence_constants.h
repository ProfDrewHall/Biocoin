/**
 * @file sequence_constants.h
 * @brief Shared AD5940 sequencer constants used by voltammetry techniques.
 */

#pragma once

#include <cstdint>

namespace sensor {
  constexpr uint32_t kSeqLenOneStep = 4UL;
  constexpr uint32_t kCurrBlk0 = 0UL;
  constexpr uint32_t kCurrBlk1 = 1UL;
} // namespace sensor
