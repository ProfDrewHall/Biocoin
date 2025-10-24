#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"

int32_t AD5940_ConfigureClock() {
  CLKCfg_Type clk_cfg = {0};
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);

  return 0;
}

int32_t AD5940_ConfigureFIFO(uint32_t FIFO_Size, uint32_t FIFO_Src) {
  FIFOCfg_Type fifo_cfg = {};
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;   /*set to DFT here first to allow for RTIA calibration*/
  fifo_cfg.FIFOThresh = 4;   // during CA, set to AppAMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another
                             // for Rz. One DFT result have real part and imaginary part */;
  AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

  return 0;
}

int32_t AD5940_ConfigureSequencer(uint32_t SEQ_MemSize) {
  /* Configure sequencer and stop it */
  SEQCfg_Type seq_cfg = {0};
  seq_cfg.SeqMemSize = SEQ_MemSize;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  return 0;
}

int32_t AD5940_ConfigureInterrupts(uint32_t interrupts) {
  AD5940_INTCCfg(AFEINTC_0, interrupts, bTRUE); // Interrupt Controller 0 will control GP0 to generate interrupt to MCU
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); // Enable all interrupts in Interrupt Controller 1, so we can check INTC flags
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  return 0;
}

int32_t AD5940_ConfigureGPIO() {
  AGPIOCfg_Type gpio_cfg = {0};
  gpio_cfg.FuncSet = GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin5 | AGPIO_Pin6;
  gpio_cfg.OutVal = 0; // check this
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);

  return 0;
}

int32_t AD5940_MeasureLFOSC(float* pFreq) {
  LFOSCMeasure_Type LfoscMeasure = {0};
  LfoscMeasure.CalDuration = 1000.0; /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = SYS_CLOCK_FREQ / 4; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, pFreq);

  return 0;
}

int32_t AD5940_CalibrateRTIA(float AdcClkFreq, float SysClkFreq, uint32_t LptiaRtiaSel, float RcalVal,
                             fImpPol_Type* RtiaCalValue) {
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE; /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = AdcClkFreq;
  lprtia_cal.SysClkFreq = SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22; /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;  /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AdcClkFreq / 4 / 22 / 2048 * 3; /* Sample 3 period of signal, 13.317Hz here. Do not use DC method,
                                                        because it needs ADC/PGA calibrated first (but it's faster) */
  lprtia_cal.fRcal = RcalVal;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  lprtia_cal.LpTiaRtia = LptiaRtiaSel;

  return AD5940_LPRtiaCal(&lprtia_cal, RtiaCalValue);
}

void AD5940_ConfigureAFEReferences(bool use_lp_bandgap, bool use_lp_refbuf, bool use_lp1v1, bool use_lp1v8) {
  AFERefCfg_Type aferef_cfg = {};
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = use_lp1v1 ? bTRUE : bFALSE;
  aferef_cfg.Lp1V8BuffEn = use_lp1v8 ? bTRUE : bFALSE;
  aferef_cfg.LpBandgapEn = use_lp_bandgap ? bTRUE : bFALSE;
  aferef_cfg.LpRefBufEn = use_lp_refbuf ? bTRUE : bFALSE;
  aferef_cfg.LpRefBoostEn = bFALSE;

  AD5940_REFCfgS(&aferef_cfg);
}

void AD5940_ConfigureLPLoop(float Vzero, float sensorBias, float TIA_Rf, float TIA_Rl, bool bExtRtia,
                            uint32_t LptiaRtiaSel) {
  LPLoopCfg_Type lp_loop = {};

  // Configure the low power DAC
  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;

  // do not send VBIAS to Pin due to it being loaded by iontophoresis block
  // Don't connect Vzero2pin either if you want to maintain fast transient of the step voltage
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | /*LPDACSW_VBIAS2PIN|*/ LPDACSW_VZERO2LPTIA /*|LPDACSW_VZERO2PIN*/;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((Vzero - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB); // WE voltage
  lp_loop.LpDacCfg.DacData12Bit =
      (int32_t)(lp_loop.LpDacCfg.DacData6Bit * 64 - (sensorBias) / AD5940_12BIT_DAC_1LSB); // RE voltage

  if (lp_loop.LpDacCfg.DacData12Bit < lp_loop.LpDacCfg.DacData6Bit * 64) lp_loop.LpDacCfg.DacData12Bit--;
  // truncate if needed
  if (lp_loop.LpDacCfg.DacData12Bit > 4095) lp_loop.LpDacCfg.DacData12Bit = 4095;
  if (lp_loop.LpDacCfg.DacData6Bit > 63) lp_loop.LpDacCfg.DacData6Bit = 63;

  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = TIA_Rf;
  lp_loop.LpAmpCfg.LpTiaRload = TIA_Rl;
  if (bExtRtia == bTRUE) {
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(9) | LPTIASW(2) | LPTIASW(4) | LPTIASW(5) /*|LPTIASW(12)*/ /*|LPTIASW(13)*/;
  } else {
    lp_loop.LpAmpCfg.LpTiaRtia = LptiaRtiaSel;
    // close SW5 to connect to external CTIA to ensure stability across different electrode double layer capacitances
    // Don't close SW13 if you want to maintain fast transient on the step voltage
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(2) | LPTIASW(4) /*|LPTIASW(12)|LPTIASW(13)*/;
  }
  AD5940_LPLoopCfgS(&lp_loop);
}

void AD5940_ConfigureDSP(uint32_t muxNsrc, uint32_t muxPsrc, uint32_t ADCPgaGain, uint32_t ADCSinc2Osr, uint32_t ADCSinc3Osr) {
  DSPCfg_Type dsp_cfg = {};
  dsp_cfg.ADCBaseCfg.ADCMuxN = muxNsrc;
  dsp_cfg.ADCBaseCfg.ADCMuxP = muxPsrc;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPgaGain;
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16; // It's disabled. Averaging only done if using DFT.
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; 
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;

  AD5940_DSPCfgS(&dsp_cfg);
}