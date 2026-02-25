/**
 * @file technique_runtime_helpers.h
 * @brief Shared runtime helpers for AD5940-based sensor techniques.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/techniques/setup_measurement_helpers.h"

#include <vector>

namespace sensor::techniqueRuntime {
  inline uint32_t drainFifoToBuffer(std::vector<uint32_t>& buffer, uint32_t sampleGranularity = 1u) {
    if (sampleGranularity == 0u) sampleGranularity = 1u;
    const uint32_t rawSamples = AD5940_FIFOGetCnt();
    const uint32_t numSamples = (rawSamples / sampleGranularity) * sampleGranularity;
    if (numSamples == 0u) return 0u;
    const size_t oldSize = buffer.size();
    buffer.resize(oldSize + numSamples);
    AD5940_FIFORd(buffer.data() + oldSize, numSamples);
    return numSamples;
  }

  inline bool stopAndPowerDown() {
    Start_AD5940_SPI();
    if (sensor::setupMeasurement::wakeupAFE() != AD5940ERR_OK) return false;
    AD5940_WUPTCtrl(bFALSE);
    AD5940_WUPTCtrl(bFALSE);
    AD5940_ShutDownS();
    Stop_AD5940_SPI();
    power::powerOffPeripherals();
    return true;
  }

  template <typename OnSample>
  inline bool forEachAdcCode(uint32_t* pData, uint32_t numSamples, OnSample onSample) {
    if (pData == nullptr) return false;
    for (uint32_t i = 0; i < numSamples; ++i) {
      onSample(pData[i] & 0xffffu);
    }
    return true;
  }

  template <typename OnCustomInt0>
  inline bool handleInterruptAndDrain(std::vector<uint32_t>& outBuffer, OnCustomInt0 onCustomInt0,
                                      uint32_t sampleGranularity = 1u) {
    bool reachedEndSequence = false;

    Start_AD5940_SPI();
    AD5940_ReadReg(REG_AFE_ADCDAT); // Any SPI operation can wakeup AFE.
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);

    const uint32_t interruptFlag = AD5940_INTCGetFlag(AFEINTC_0);
    if ((interruptFlag & AFEINTSRC_CUSTOMINT0) != 0u) {
      AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
      onCustomInt0();
      AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
      AD5940_EnterSleepS(); // If there is need to re-configure AFE, do it while active.
    }

    if ((interruptFlag & AFEINTSRC_DATAFIFOTHRESH) != 0u) {
      drainFifoToBuffer(outBuffer, sampleGranularity);
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    }

    if ((interruptFlag & AFEINTSRC_ENDSEQ) != 0u) {
      drainFifoToBuffer(outBuffer, sampleGranularity);
      AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
      reachedEndSequence = true;
    }

    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    if (!reachedEndSequence) AD5940_EnterSleepS();

    Stop_AD5940_SPI();
    return reachedEndSequence;
  }

  inline bool handleDataFifoThresholdOnly(std::vector<uint32_t>& outBuffer, uint32_t sampleGranularity = 1u) {
    bool drainedData = false;

    Start_AD5940_SPI();
    AD5940_ReadReg(REG_AFE_ADCDAT); // Any SPI operation can wakeup AFE.
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);

    const uint32_t interruptFlag = AD5940_INTCGetFlag(AFEINTC_0);
    if ((interruptFlag & AFEINTSRC_DATAFIFOTHRESH) != 0u) {
      drainFifoToBuffer(outBuffer, sampleGranularity);
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
      drainedData = true;
      AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
      AD5940_EnterSleepS();
    } else {
      AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    }

    Stop_AD5940_SPI();
    return drainedData;
  }
} // namespace sensor::techniqueRuntime
