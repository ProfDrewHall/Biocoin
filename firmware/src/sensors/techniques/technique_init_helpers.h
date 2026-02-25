/**
 * @file technique_init_helpers.h
 * @brief Shared AD5940 baseline initialization helper for sensor techniques.
 */

#pragma once

#include "drivers/ad5940_hal.h"

namespace sensor::techniqueInit {
  struct AD5940InitConfig {
    uint32_t fifoSize;
    uint32_t fifoSource;
    bool configureSequencer;
    uint32_t sequencerMemSize;
    uint32_t interruptFlags;
  };

  inline int32_t initializeAD5940(const AD5940InitConfig& config, float* lfoscFreq) {
    AD5940_HWReset();
    AD5940_Initialize();
    AD5940_ConfigureClock();
    AD5940_ConfigureFIFO(config.fifoSize, config.fifoSource);
    if (config.configureSequencer) AD5940_ConfigureSequencer(config.sequencerMemSize);
    AD5940_ConfigureInterrupts(config.interruptFlags);
    AD5940_ConfigureGPIO();
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    if (lfoscFreq != nullptr) AD5940_MeasureLFOSC(lfoscFreq);
    return 0;
  }
} // namespace sensor::techniqueInit
