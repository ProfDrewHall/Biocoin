/**
 * @file echem_ca_sequence.cpp
 * @brief Chronoamperometry (CA) sequencer program generation.
 */

#include "sensors/techniques/ca/echem_ca.h"

// Generate one-time CA initialization sequence.
AD5940Err sensor::EChem_CA::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Initialize all blocks to disabled state.

  AD5940_ConfigureAFEReferences(true, true, true, true);
  AD5940_ConfigureLPLoop(config.Vzero, config.SensorBias, config.LpTiaRf, config.LpTiaRl, config.ExtRtia,
                         config.LptiaRtiaSel);
  AD5940_ConfigureDSP(ADCMUXN_LPTIA0_N, ADCMUXP_LPTIA0_P, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

  HSLoopCfg_Type hsLoop = {0};
  AD5940_HSLoopCfgS(&hsLoop);
  // Enable required blocks. Hardware powers these down in hibernate automatically.
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0);

  // Stop sequencer after one init pass.
  AD5940_SEQGenInsert(SEQ_STOP());
  AD5940_SEQGenCtrl(bFALSE);

  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  if (error == AD5940ERR_OK) {
    config.InitSeqInfo.SeqId = SEQID_1;
    config.InitSeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.InitSeqInfo.pSeqCmd = seqCmd;
    config.InitSeqInfo.SeqLen = seqLen;
    AD5940_SEQCmdWrite(config.InitSeqInfo.SeqRamAddr, seqCmd, seqLen);
  } else {
    return error;
  }
  return AD5940ERR_OK;
}

// Generate recurring CA measurement sequence.
AD5940Err sensor::EChem_CA::generateMeasurementSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;
  uint32_t waitClks;
  ClksCalInfo_Type clksCal;

  clksCal.DataType = config.DataFifoSrc;
  clksCal.DataCount = 1u;
  clksCal.ADCSinc2Osr = config.ADCSinc2Osr;
  clksCal.ADCSinc3Osr = config.ADCSinc3Osr;
  clksCal.ADCAvgNum = 0u;
  clksCal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clksCal, &waitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16u * kCASeqRefSettleUs));
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE);
  AD5940_SEQGenInsert(SEQ_WAIT(kCASeqPostAdcDelayClks));
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if (error == AD5940ERR_OK) {
    config.MeasureSeqInfo.SeqId = SEQID_0;
    config.MeasureSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
    config.MeasureSeqInfo.pSeqCmd = seqCmd;
    config.MeasureSeqInfo.SeqLen = seqLen;
    AD5940_SEQCmdWrite(config.MeasureSeqInfo.SeqRamAddr, seqCmd, seqLen);
  } else {
    return error;
  }
  return AD5940ERR_OK;
}
