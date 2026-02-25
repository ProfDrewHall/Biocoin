/**
 * @file iontophoresis_config.cpp
 * @brief Iontophoresis AD5940 initialization and DAC setup.
 */

#include "sensors/techniques/iontophoresis/iontophoresis.h"
#include "sensors/techniques/technique_init_helpers.h"

#include "HWConfig/constants.h"

int32_t sensor::Iontophoresis::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_DFT, true, SEQMEMSIZE_2KB, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ}, &LFOSCFreq);
}

AD5940Err sensor::Iontophoresis::configureDAC(void) {
  float voltage =
      config.stimCurrent * config.Rsense * config.Av_CSA / 1000; // Convert desired current into a DAC voltage in [mV]

  AD5940_ConfigureAFEReferences(true, true, false, false);
  LPLoopCfg_Type lp_cfg = {0};
  lp_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lp_cfg.LpDacCfg.DacData12Bit = (voltage - AD5940_MIN_DAC_OUTPUT) / AD5940_12BIT_DAC_1LSB;
  lp_cfg.LpDacCfg.DacData6Bit = 0;
  lp_cfg.LpDacCfg.DataRst = bFALSE;
  lp_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2PIN; // | LPDACSW_VBIAS2LPPA;
  lp_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lp_cfg);

  return AD5940ERR_OK;
}
