/**
 * @file echem_swv_sequence.cpp
 * @brief Square-wave voltammetry (SWV) sequencer program generation.
 */

#include "sensors/techniques/swv/echem_swv.h"
#include "sensors/techniques/sequence_dac_helpers.h"
#include "sensors/techniques/sequence_constants.h"

#include "HWConfig/constants.h"

AD5940Err sensor::EChem_SWV::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all blocks to disabled state.

  AD5940_ConfigureAFEReferences(true, true, true, true);
  configureLPLoop();
  AD5940_ConfigureDSP(ADCMUXN_LPTIA0_N, ADCMUXP_LPTIA0_P, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

  HSLoopCfg_Type hsLoop = {0};
  AD5940_HSLoopCfgS(&hsLoop);

  // Required blocks are automatically turned off during hibernate mode.
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0);

  // Stop sequence after one initialization pass.
  AD5940_SEQGenInsert(SEQ_STOP());

  AD5940_SEQGenCtrl(bFALSE);
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  if (error == AD5940ERR_OK) {
    AD5940_StructInit(&config.InitSeqInfo, sizeof(config.InitSeqInfo));
    if (seqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN;

    config.InitSeqInfo.SeqId = SEQID_3;
    config.InitSeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.InitSeqInfo.pSeqCmd = seqCmd;
    config.InitSeqInfo.SeqLen = seqLen;
    config.InitSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&config.InitSeqInfo);
  } else {
    return error;
  }
  return AD5940ERR_OK;
}


AD5940Err sensor::EChem_SWV::generateADCSequence(void) {
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
  AD5940_SEQGenInsert(SEQ_WAIT(16u * kSWVSeqRefSettleUs));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGenInsert(SEQ_WAIT(kSWVSeqPostAdcDelayClks));
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  AD5940_SEQGenCtrl(bFALSE);
  if (error != AD5940ERR_OK) return error;

  AD5940_StructInit(&config.ADCSeqInfo, sizeof(config.ADCSeqInfo));
  if (seqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN;

  config.ADCSeqInfo.SeqId = SEQID_2;
  config.ADCSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
  config.ADCSeqInfo.pSeqCmd = seqCmd;
  config.ADCSeqInfo.SeqLen = seqLen;
  config.ADCSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&config.ADCSeqInfo);

  return AD5940ERR_OK;
}


AD5940Err sensor::EChem_SWV::generateDACSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t blockStartSramAddr;
  uint32_t dacData, sramAddr;
  uint32_t i;
  uint32_t stepsThisBlock;
  BoolFlag isFinalBlock;

  const float dir = (config.Estop > config.Estart) ? -1.0f : +1.0f; // because Vbias = -Ewe
  const float stepAbs = fabsf(config.Estep);
  const float pulseAbs = fabsf(config.RampAmplitudeMv);
  config.StepNumber = (uint32_t)(2.0f * round((fabsf(config.RampPeakVolt - config.RampStartVolt) / stepAbs)));

  // Do sequence/dac math once at the beginning.
  if (config.bFirstDACSeq == bTRUE) {
    int32_t dacSeqLenMax;
    dacSeqLenMax = (int32_t)config.MaxSeqLen - (int32_t)config.InitSeqInfo.SeqLen - (int32_t)config.ADCSeqInfo.SeqLen;
    if (!sequenceDac::initializeBlockState(dacSeqLenMax, config.StepNumber,
                                           config.ADCSeqInfo.SeqRamAddr + config.ADCSeqInfo.SeqLen, stepsRemaining,
                                           stepsPerBlock, dacSeqBlk0Addr, dacSeqBlk1Addr, dacSeqCurrBlk))
      return AD5940ERR_SEQLEN;

    // Analog part.
    config.DACCodePerStep = dir * (pulseAbs / AD5940_12BIT_DAC_1LSB);
    config.DACCodePerRamp = dir * (stepAbs / AD5940_12BIT_DAC_1LSB);

#if ALIGIN_VOLT2LSB
    config.DACCodePerStep = (int32_t)config.DACCodePerStep;
    config.DACCodePerRamp = (int32_t)config.DACCodePerRamp;
#endif

    config.CurrRampCode = config.RampStartVolt / AD5940_12BIT_DAC_1LSB;

    rampState = SWVRampState::Start;
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
