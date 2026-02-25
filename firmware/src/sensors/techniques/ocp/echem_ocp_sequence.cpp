/**
 * @file echem_ocp_sequence.cpp
 * @brief Open-circuit potential (OCP) sequencer program generation.
 */

#include "sensors/techniques/ocp/echem_ocp.h"
#include "sensors/techniques/lploop_helpers.h"
#include "sensors/techniques/single_measurement_sequence_helpers.h"

#include "HWConfig/constants.h"

AD5940Err sensor::EChem_OCP::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Initialize all blocks to disabled state.

  AD5940_ConfigureAFEReferences(true, true, true, true);

  lpLoop::configureOpenCircuitLoop(config.Vzero, config.Vbias);

  AD5940_ConfigureDSP(ADCMUXN_VBIAS0, channel, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

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

// Generate recurring OCP measurement sequence.
AD5940Err sensor::EChem_OCP::generateMeasurementSequence(void) {
  const uint32_t* seqCmd;
  uint32_t seqLen;
  const AD5940Err error = singleMeasurementSequence::generate(
      {config.DataFifoSrc, config.ADCSinc2Osr, config.ADCSinc3Osr, config.SysClkFreq, config.AdcClkFreq,
       kOCPSeqRefSettleUs, kOCPSeqPostAdcDelayClks},
      &seqCmd, &seqLen);

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
