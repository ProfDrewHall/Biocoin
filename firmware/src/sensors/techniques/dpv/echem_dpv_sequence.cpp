/**
 * @file echem_dpv_sequence.cpp
 * @brief Differential pulse voltammetry (DPV) sequencer program generation.
 */

#include "sensors/techniques/dpv/echem_dpv.h"
#include "sensors/techniques/sequence_dac_helpers.h"
#include "sensors/techniques/sequence_constants.h"

#include "HWConfig/constants.h"

AD5940Err sensor::EChem_DPV::generateInitSequence(void) {
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all blocks to disabled state.
  AD5940_ConfigureAFEReferences(true, true, false, false);
  configureLPLoop();
  AD5940_ConfigureDSP(ADCMUXN_LPTIA0_N, ADCMUXP_LPTIA0_P, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

  HSLoopCfg_Type hsLoop = {0};
  AD5940_HSLoopCfgS(&hsLoop);

  // Required blocks are automatically turned off during hibernate mode.
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);

  return AD5940ERR_OK;
}

// ADC sampling sequence for the high part of the DPV pulse.
AD5940Err sensor::EChem_DPV::generateADCSequenceHigh(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;
  uint32_t waitClks;
  ClksCalInfo_Type clksCal;

  clksCal.DataCount = 1u;
  clksCal.DataType = config.DataFifoSrc;
  clksCal.ADCSinc3Osr = config.ADCSinc3Osr;
  clksCal.ADCSinc2Osr = config.ADCSinc2Osr;
  clksCal.ADCAvgNum = 0;
  clksCal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clksCal, &waitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16u * kDPVSeqRefSettleUs));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGenInsert(SEQ_WAIT(kDPVSeqPostAdcDelayClks));
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if (error == AD5940ERR_OK) {
    AD5940_StructInit(&config.ADC_HighPulse_SeqInfo, sizeof(config.ADC_HighPulse_SeqInfo));
    if (seqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN;
    config.ADC_HighPulse_SeqInfo.SeqId = SEQID_2;
    config.ADC_HighPulse_SeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.ADC_HighPulse_SeqInfo.pSeqCmd = seqCmd;
    config.ADC_HighPulse_SeqInfo.SeqLen = seqLen;
    config.ADC_HighPulse_SeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&config.ADC_HighPulse_SeqInfo);
  } else {
    return error;
  }
  return AD5940ERR_OK;
}

// ADC sampling sequence for the low part of the DPV pulse.
AD5940Err sensor::EChem_DPV::generateADCSequenceLow(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;
  uint32_t waitClks;
  ClksCalInfo_Type clksCal;

  clksCal.DataCount = 1u;
  clksCal.DataType = config.DataFifoSrc;
  clksCal.ADCSinc3Osr = config.ADCSinc3Osr;
  clksCal.ADCSinc2Osr = config.ADCSinc2Osr;
  clksCal.ADCAvgNum = 0;
  clksCal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clksCal, &waitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16u * kDPVSeqRefSettleUs));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGenInsert(SEQ_WAIT(kDPVSeqPostAdcDelayClks));
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  AD5940_SEQGenCtrl(bFALSE);
  if (error != AD5940ERR_OK) return error;

  AD5940_StructInit(&config.ADC_LowPulse_SeqInfo, sizeof(config.ADC_LowPulse_SeqInfo));
  if (seqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN;

  // Put LOW at the very end of the available window.
  const uint32_t lowStart = config.SeqStartAddr + (config.MaxSeqLen - seqLen);
  const uint32_t highEnd = config.ADC_HighPulse_SeqInfo.SeqRamAddr + config.ADC_HighPulse_SeqInfo.SeqLen;

  // Require non-overlap with HIGH and leave room in the middle for DAC.
  if (highEnd >= lowStart) return AD5940ERR_SEQLEN;

  // Write LOW sequence at the end.
  config.ADC_LowPulse_SeqInfo.SeqId = SEQID_3;
  config.ADC_LowPulse_SeqInfo.SeqRamAddr = lowStart;
  config.ADC_LowPulse_SeqInfo.pSeqCmd = seqCmd;
  config.ADC_LowPulse_SeqInfo.SeqLen = seqLen;
  config.ADC_LowPulse_SeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&config.ADC_LowPulse_SeqInfo);

  return AD5940ERR_OK;
}


AD5940Err sensor::EChem_DPV::generateDACSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t blockStartSramAddr;
  uint32_t dacData, sramAddr;
  uint32_t i;
  uint32_t stepsThisBlock;
  BoolFlag isFinalBlock;

  const float dir = (config.Estop > config.Estart) ? -1.0f : +1.0f; // because Vbias = -Ewe
  const float stepAbs = fabsf(config.Estep);
  const float pulseAbs = fabsf(config.Epulse);
  config.StepNumber = (uint32_t)(2.0f * round((fabsf(config.RampPeakVolt - config.RampStartVolt) / stepAbs)));

  // Do sequence/dac math once at the beginning.
  if (config.bFirstDACSeq == bTRUE) {
    int32_t dacSeqLenMax;
    dacSeqLenMax = (int32_t)config.MaxSeqLen - (int32_t)config.ADC_HighPulse_SeqInfo.SeqLen -
                   (int32_t)config.ADC_LowPulse_SeqInfo.SeqLen;
    if (!sequenceDac::initializeBlockState(dacSeqLenMax, config.StepNumber,
                                           config.ADC_HighPulse_SeqInfo.SeqRamAddr + config.ADC_HighPulse_SeqInfo.SeqLen,
                                           stepsRemaining, stepsPerBlock, dacSeqBlk0Addr, dacSeqBlk1Addr,
                                           dacSeqCurrBlk))
      return AD5940ERR_SEQLEN;

    // Analog part.
    config.DACCodePerStep = dir * (pulseAbs / AD5940_12BIT_DAC_1LSB);
    config.DACCodePerRamp = dir * (stepAbs / AD5940_12BIT_DAC_1LSB);

#if ALIGIN_VOLT2LSB
    config.DACCodePerStep = (int32_t)config.DACCodePerStep;
    config.DACCodePerRamp = (int32_t)config.DACCodePerRamp;
#endif

    config.CurrRampCode =
        config.RampStartVolt / AD5940_12BIT_DAC_1LSB + (config.DACCodePerStep - config.DACCodePerRamp);

    rampState = DPVRampState::Start;
    config.CurrStepPos = 0;
    seqCmdForSeq0 = bTRUE;
  }

  if (stepsRemaining == 0) return AD5940ERR_OK;
  isFinalBlock = stepsRemaining <= stepsPerBlock ? bTRUE : bFALSE;
  stepsThisBlock = isFinalBlock ? stepsRemaining : stepsPerBlock;
  stepsRemaining -= stepsThisBlock;

  blockStartSramAddr = (dacSeqCurrBlk == kCurrBlk0) ? dacSeqBlk0Addr : dacSeqBlk1Addr;
  sramAddr = blockStartSramAddr;

  for (i = 0; i < stepsThisBlock - 1; i++) {
    uint32_t currAddr = sramAddr;
    sramAddr += kSeqLenOneStep;
    updateRampDACCode(&dacData);
    sequenceDac::writeDacStepToSleep(currAddr, sramAddr, dacData, seqCmdForSeq0);
    seqCmdForSeq0 = sequenceDac::toggleSeq(seqCmdForSeq0);
  }

  if (isFinalBlock) {
    uint32_t currAddr = sramAddr;
    sramAddr += kSeqLenOneStep;
    updateRampDACCode(&dacData);
    sequenceDac::writeDacStepToSleep(currAddr, sramAddr, dacData, seqCmdForSeq0);
    currAddr += kSeqLenOneStep;
    sequenceDac::writeFinalStop(currAddr);
  } else {
    uint32_t currAddr = sramAddr;
    sramAddr = (dacSeqCurrBlk == kCurrBlk0) ? dacSeqBlk1Addr : dacSeqBlk0Addr;
    updateRampDACCode(&dacData);
    sequenceDac::writeDacStepToInterrupt(currAddr, sramAddr, dacData, seqCmdForSeq0);
    seqCmdForSeq0 = sequenceDac::toggleSeq(seqCmdForSeq0);
  }

  dacSeqCurrBlk = (dacSeqCurrBlk == kCurrBlk0) ? kCurrBlk1 : kCurrBlk0;

  if (config.bFirstDACSeq) {
    config.bFirstDACSeq = bFALSE;
    if (isFinalBlock == bFALSE) {
      error = generateDACSequence();
      if (error != AD5940ERR_OK) return error;
    }
    config.DACSeqInfo.SeqId = SEQID_0;
    config.DACSeqInfo.SeqLen = kSeqLenOneStep;
    config.DACSeqInfo.SeqRamAddr = blockStartSramAddr;
    config.DACSeqInfo.WriteSRAM = bFALSE; // Commands were written above.
    AD5940_SEQInfoCfg(&config.DACSeqInfo);
  }

  return AD5940ERR_OK;
}
