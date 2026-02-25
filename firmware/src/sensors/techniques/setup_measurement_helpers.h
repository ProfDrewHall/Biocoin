/**
 * @file setup_measurement_helpers.h
 * @brief Shared AD5940 setup helpers for technique measurement initialization.
 */

#pragma once

#include "drivers/ad5940_hal.h"

#include <cstddef>
#include <initializer_list>

namespace sensor::setupMeasurement {
  constexpr uint32_t kWakeupAttempts = 10u;

  struct SetupOptions {
    uint32_t seqMemSize;
    uint32_t fifoDisableSource;
    uint32_t fifoSource;
    uint32_t fifoSize;
  };

  inline AD5940Err wakeupAFE() {
    if (AD5940_WakeUp(kWakeupAttempts) > kWakeupAttempts) return AD5940ERR_WAKEUP;
    return AD5940ERR_OK;
  }

  inline void configureSequencerStopped(uint32_t seqMemSize) {
    SEQCfg_Type seqCfg = {};
    seqCfg.SeqMemSize = seqMemSize;
    seqCfg.SeqBreakEn = bFALSE;
    seqCfg.SeqIgnoreEn = bFALSE;
    seqCfg.SeqCntCRCClr = bTRUE;
    seqCfg.SeqEnable = bFALSE;
    seqCfg.SeqWrTimer = 0;
    AD5940_SEQCfg(&seqCfg);
  }

  inline void configureDataFifo(uint32_t fifoDisableSource, uint32_t fifoSource, uint32_t fifoThreshold,
                                uint32_t fifoSize) {
    AD5940_FIFOCtrlS(fifoDisableSource, bFALSE);

    FIFOCfg_Type fifoCfg = {};
    fifoCfg.FIFOEn = bTRUE;
    fifoCfg.FIFOSrc = fifoSource;
    fifoCfg.FIFOThresh = fifoThreshold;
    fifoCfg.FIFOMode = FIFOMODE_FIFO;
    fifoCfg.FIFOSize = fifoSize;
    AD5940_FIFOCfg(&fifoCfg);
  }

  inline AD5940Err beginSetup(uint32_t* seqBuffer, size_t seqBufferSize, uint32_t fifoThreshold,
                              const SetupOptions& options) {
    if (seqBuffer == nullptr || seqBufferSize == 0u) return AD5940ERR_PARA;

    const AD5940Err wakeError = wakeupAFE();
    if (wakeError != AD5940ERR_OK) return wakeError;

    configureSequencerStopped(options.seqMemSize);
    configureDataFifo(options.fifoDisableSource, options.fifoSource, fifoThreshold, options.fifoSize);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    AD5940_SEQGenInit(seqBuffer, seqBufferSize);
    return AD5940ERR_OK;
  }

  inline AD5940Err rearmAfterCalibration(uint32_t* seqBuffer, size_t seqBufferSize, uint32_t fifoThreshold,
                                         const SetupOptions& options) {
    if (seqBuffer == nullptr || seqBufferSize == 0u) return AD5940ERR_PARA;

    configureDataFifo(options.fifoDisableSource, options.fifoSource, fifoThreshold, options.fifoSize);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    AD5940_SEQGenInit(seqBuffer, seqBufferSize);
    return AD5940ERR_OK;
  }

  inline void triggerInitSequenceBlocking(SEQInfo_Type& initSeqInfo) {
    initSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&initSeqInfo);

    AD5940_SEQCtrlS(bTRUE);
    AD5940_SEQMmrTrig(initSeqInfo.SeqId);
    while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE) {
    }
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  }

  inline void armSequencerForWakeup() {
    AD5940_SEQCtrlS(bFALSE);
    AD5940_WriteReg(REG_AFE_SEQCNT, 0);
    AD5940_SEQCtrlS(bTRUE);
  }

  struct SetupPipeline {
    uint32_t* seqBuffer;
    size_t seqBufferSize;
    uint32_t fifoThreshold;
    SetupOptions options;

    AD5940Err begin() const {
      return setupMeasurement::beginSetup(seqBuffer, seqBufferSize, fifoThreshold, options);
    }

    AD5940Err rearmAfterCalibration() const {
      return setupMeasurement::rearmAfterCalibration(seqBuffer, seqBufferSize, fifoThreshold, options);
    }

    void triggerInitBlocking(SEQInfo_Type& initSeqInfo) const {
      setupMeasurement::triggerInitSequenceBlocking(initSeqInfo);
    }

    void publishSequenceNoSram(SEQInfo_Type& seqInfo) const {
      seqInfo.WriteSRAM = bFALSE;
      AD5940_SEQInfoCfg(&seqInfo);
    }

    void publishSequencesNoSram(std::initializer_list<SEQInfo_Type*> seqInfos) const {
      for (SEQInfo_Type* seq : seqInfos) {
        if (seq == nullptr) continue;
        publishSequenceNoSram(*seq);
      }
    }

    void armForWakeup() const {
      setupMeasurement::armSequencerForWakeup();
    }
  };
} // namespace sensor::setupMeasurement
