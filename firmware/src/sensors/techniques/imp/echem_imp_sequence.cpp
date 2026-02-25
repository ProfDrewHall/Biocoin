/**
 * @file echem_imp_sequence.cpp
 * @brief Impedance (IMP) sequencer program generation.
 */

#include "sensors/techniques/imp/echem_imp.h"

#include "HWConfig/constants.h"
#include "util/debug_log.h"

AD5940Err sensor::EChem_Imp::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;
  float sinFreq;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Initialize all blocks to disabled state.

  const bool lpDacAndTiaNeeded = config.imp4Wire && config.acCoupled;
  AD5940_ConfigureAFEReferences(lpDacAndTiaNeeded, lpDacAndTiaNeeded, false, false);

  HSLoopCfg_Type hsLoop = {0};
  hsLoop.HsDacCfg.ExcitBufGain = config.ExcitBufGain;
  hsLoop.HsDacCfg.HsDacGain = config.HsDacGain;
  hsLoop.HsDacCfg.HsDacUpdateRate = config.HsDacUpdateRate;

  hsLoop.HsTiaCfg.DiodeClose = bFALSE;
  hsLoop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsLoop.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hsLoop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsLoop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hsLoop.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;

  hsLoop.SWMatCfg.Dswitch = SWD_OPEN;
  hsLoop.SWMatCfg.Pswitch = SWP_PL | SWP_PL2;
  hsLoop.SWMatCfg.Nswitch = SWN_NL | SWN_NL2;
  hsLoop.SWMatCfg.Tswitch = SWT_TRTIA;

  hsLoop.WgCfg.WgType = WGTYPE_SIN;
  hsLoop.WgCfg.GainCalEn = bFALSE;
  hsLoop.WgCfg.OffsetCalEn = bFALSE;
  if (config.SweepCfg.SweepEn == bTRUE) {
    config.SweepCfg.SweepIndex = 0;
    config.FreqofData = config.SweepCfg.SweepStart;
    config.SweepCurrFreq = config.SweepCfg.SweepStart;
    AD5940_SweepNext(&config.SweepCfg, &config.SweepNextFreq);
    sinFreq = config.SweepCurrFreq;
  } else {
    sinFreq = config.SinFreq;
    config.FreqofData = sinFreq;
  }
  hsLoop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sinFreq, config.SysClkFreq);
  hsLoop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(config.DacVoltPP / 800.0f * 2047 + 0.5f);
  hsLoop.WgCfg.SinCfg.SinOffsetWord = 0;
  hsLoop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hsLoop);

  if (lpDacAndTiaNeeded) {
    LPLoopCfg_Type lpLoop = {0};

    lpLoop.LpDacCfg.LpdacSel = LPDAC0;
    lpLoop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lpLoop.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA;
    lpLoop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lpLoop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lpLoop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lpLoop.LpDacCfg.DataRst = bFALSE;
    lpLoop.LpDacCfg.PowerEn = bTRUE;
    lpLoop.LpDacCfg.DacData6Bit = (uint32_t)((1300 - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    lpLoop.LpDacCfg.DacData12Bit = (int32_t)(lpLoop.LpDacCfg.DacData6Bit * 64);

    lpLoop.LpAmpCfg.LpAmpSel = LPAMP0;
    lpLoop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_HALF;
    lpLoop.LpAmpCfg.LpPaPwrEn = bFALSE;
    lpLoop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lpLoop.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lpLoop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
    lpLoop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lpLoop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(6) | LPTIASW(7) | LPTIASW(9);
    AD5940_LPLoopCfgS(&lpLoop);
  }

  DSPCfg_Type dspCfg = {0};
  dspCfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dspCfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dspCfg.ADCBaseCfg.ADCPga = config.ADCPgaGain;

  memset(&dspCfg.ADCDigCompCfg, 0, sizeof(dspCfg.ADCDigCompCfg));

  dspCfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;
  dspCfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dspCfg.ADCFilterCfg.ADCSinc2Osr = config.ADCSinc2Osr;
  dspCfg.ADCFilterCfg.ADCSinc3Osr = config.ADCSinc3Osr;
  dspCfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dspCfg.ADCFilterCfg.BpNotch = bTRUE;
  dspCfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dspCfg.DftCfg.DftNum = config.DftNum;
  dspCfg.DftCfg.DftSrc = config.DftSrc;
  dspCfg.DftCfg.HanWinEn = config.HanWinEn;

  memset(&dspCfg.StatCfg, 0, sizeof(dspCfg.StatCfg));
  AD5940_DSPCfgS(&dspCfg);

  // Enable required blocks. Hardware powers these down in hibernate automatically.
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_HSTIAPWR | AFECTRL_INAMPPWR | AFECTRL_EXTBUFPWR | AFECTRL_WG |
                      AFECTRL_DACREFPWR | AFECTRL_HSDACPWR | AFECTRL_SINC2NOTCH,
                  bTRUE);

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

// Generate recurring IMP measurement sequence.
AD5940Err sensor::EChem_Imp::generateMeasurementSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* seqCmd;
  uint32_t seqLen;

  uint32_t waitClks;
  SWMatrixCfg_Type swCfg;
  ClksCalInfo_Type clksCal;

  clksCal.DataType = DATATYPE_DFT;
  clksCal.DftSrc = config.DftSrc;
  clksCal.DataCount = 1L << (config.DftNum + 2);
  clksCal.ADCSinc2Osr = config.ADCSinc2Osr;
  clksCal.ADCSinc3Osr = config.ADCSinc3Osr;
  clksCal.ADCAvgNum = 0;
  clksCal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clksCal, &waitClks);

  AD5940_SEQGenCtrl(bTRUE);

  AD5940_SEQGenInsert(SEQ_WAIT(16 * kIMPSeqRefSettleUs)); // Wait for reference power-up settling.
  swCfg.Dswitch = config.DswitchSel;
  swCfg.Pswitch = config.PswitchSel;
  swCfg.Nswitch = config.NswitchSel;
  swCfg.Tswitch = config.TswitchSel | SWT_TRTIA;
  AD5940_SWMatrixCfgS(&swCfg);

  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE);

  if (config.imp4Wire)
    AD5940_ADCMuxCfgS(config.SenseP, config.SenseN);
  else
    AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_N_NODE);

  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50)); // Settling delay.
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(waitClks));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE);

  swCfg.Dswitch = SWD_OPEN;
  swCfg.Pswitch = SWP_PL | SWP_PL2;
  swCfg.Nswitch = SWN_NL | SWN_NL2;
  swCfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&swCfg);
  AD5940_EnterSleepS();
  error = AD5940_SEQGenFetchSeq(&seqCmd, &seqLen);
  AD5940_SEQGenCtrl(bFALSE);

  config.MeasSeqCycleCount = AD5940_SEQCycleTime();
  config.MaxODR = 1 / (((config.MeasSeqCycleCount + 10) / 16.0) * 1E-6);
  if (config.SamplingIntervalS > config.MaxODR) {
    // Clamp to the fastest rate achievable for this sequence timing.
    config.SamplingIntervalS = config.MaxODR;
  }

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

AD5940Err sensor::EChem_Imp::AD5940_CalibrateHSRTIA(void) {
  HSRTIACal_Type hsrtiaCal;

  hsrtiaCal.AdcClkFreq = config.AdcClkFreq;
  hsrtiaCal.ADCSinc2Osr = config.ADCSinc2Osr;
  hsrtiaCal.ADCSinc3Osr = config.ADCSinc3Osr;
  hsrtiaCal.bPolarResult = bTRUE;
  hsrtiaCal.DftCfg.DftNum = config.DftNum;
  hsrtiaCal.DftCfg.DftSrc = config.DftSrc;
  hsrtiaCal.DftCfg.HanWinEn = config.HanWinEn;
  hsrtiaCal.fRcal = config.RcalVal;
  hsrtiaCal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtiaCal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtiaCal.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hsrtiaCal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtiaCal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  hsrtiaCal.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;
  hsrtiaCal.SysClkFreq = config.SysClkFreq;
  hsrtiaCal.fFreq = config.SweepCfg.SweepStart;

  if (config.SweepCfg.SweepEn == bTRUE) {
    uint32_t i;
    config.SweepCfg.SweepIndex = 0;
    for (i = 0; i < config.SweepCfg.SweepPoints; i++) {
      AD5940_HSRtiaCal(&hsrtiaCal, config.RtiaCalTable[i]);
      dbgInfo(String("Freq: ") + String(hsrtiaCal.fFreq) + String(", RTIA: Mag: ") +
              String(config.RtiaCalTable[i][0]) + String(" Ohm, Phase: ") + String(config.RtiaCalTable[i][1]));
      AD5940_SweepNext(&config.SweepCfg, &hsrtiaCal.fFreq);
    }
    config.SweepCfg.SweepIndex = 0;
    config.RtiaCurrValue[0] = config.RtiaCalTable[config.SweepCfg.SweepIndex][0];
    config.RtiaCurrValue[1] = config.RtiaCalTable[config.SweepCfg.SweepIndex][1];
  } else {
    hsrtiaCal.fFreq = config.SinFreq;
    AD5940_HSRtiaCal(&hsrtiaCal, config.RtiaCurrValue);
  }
  return AD5940ERR_OK;
}
