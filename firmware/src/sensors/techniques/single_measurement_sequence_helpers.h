/**
 * @file single_measurement_sequence_helpers.h
 * @brief Shared sequencer generation for single-point ADC measurements.
 */

#pragma once

#include "drivers/ad5940_hal.h"

namespace sensor::singleMeasurementSequence {
  struct Config {
    uint32_t dataType;
    uint8_t adcSinc2Osr;
    uint8_t adcSinc3Osr;
    float sysClkFreq;
    float adcClkFreq;
    uint32_t refSettleUs;
    uint32_t postAdcDelayClks;
  };

  inline AD5940Err generate(const Config& cfg, uint32_t const** outSeqCmd, uint32_t* outSeqLen) {
    if (outSeqCmd == nullptr || outSeqLen == nullptr) return AD5940ERR_PARA;

    uint32_t waitClks = 0u;
    ClksCalInfo_Type clksCal = {};
    clksCal.DataType = cfg.dataType;
    clksCal.DataCount = 1u;
    clksCal.ADCSinc2Osr = cfg.adcSinc2Osr;
    clksCal.ADCSinc3Osr = cfg.adcSinc3Osr;
    clksCal.ADCAvgNum = 0;
    clksCal.RatioSys2AdcClk = cfg.sysClkFreq / cfg.adcClkFreq;
    AD5940_ClksCalculate(&clksCal, &waitClks);

    AD5940_SEQGenCtrl(bTRUE);
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16u * cfg.refSettleUs));
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE);
    AD5940_SEQGenInsert(SEQ_WAIT(cfg.postAdcDelayClks));
    AD5940_EnterSleepS();

    const AD5940Err error = AD5940_SEQGenFetchSeq(outSeqCmd, outSeqLen);
    AD5940_SEQGenCtrl(bFALSE);
    return error;
  }
} // namespace sensor::singleMeasurementSequence
