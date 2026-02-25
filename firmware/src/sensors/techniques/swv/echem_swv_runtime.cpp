/**
 * @file echem_swv_runtime.cpp
 * @brief Square-wave voltammetry (SWV) runtime data processing and ramp updates.
 */

#include "sensors/techniques/swv/echem_swv.h"

#include "HWConfig/constants.h"

AD5940Err sensor::EChem_SWV::updateRampDACCode(uint32_t* pDACData) {
  if (pDACData == nullptr) return AD5940ERR_PARA;

  // 1) Handle state transitions (only when thresholds are crossed)
  switch (rampState) {
  case SWVRampState::Start:
    config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    rampState = SWVRampState::State1;
    break;
  case SWVRampState::State1:
    if (config.CurrStepPos >= config.StepNumber / 2) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroPeak - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      rampState = SWVRampState::State2;
    }
    break;
  case SWVRampState::State2:
    if (config.CurrStepPos >= config.StepNumber) rampState = SWVRampState::Stop;
    break;
  case SWVRampState::Stop:
    break;
  }

  // 2) Advance step position and ramp code
  if (config.bSqrWaveHiLevel) {
    config.CurrRampCode -= (config.DACCodePerStep - config.DACCodePerRamp);
    config.bSqrWaveHiLevel = bFALSE;
  } else {
    if (config.CurrStepPos == 1) { // first DAC rising edge uses half-step
      config.CurrRampCode += config.DACCodePerStep / 2;
    } else {
      config.CurrRampCode += config.DACCodePerStep;
    }
    config.bSqrWaveHiLevel = bTRUE;
  }

  config.CurrStepPos++;

  // 3) Compose VZERO (6b) + VBIAS (12b), with saturation
  uint32_t VzeroCode = config.CurrVzeroCode;
  uint32_t VbiasCode = (uint32_t)(VzeroCode * 64 + config.CurrRampCode);
  if (VbiasCode < (VzeroCode * 64)) VbiasCode--;

  /* Truncate */
  if (VbiasCode > 4095) VbiasCode = 4095;
  if (VzeroCode > 63) VzeroCode = 63;

  *pDACData = (VzeroCode << 12) | VbiasCode;
  return AD5940ERR_OK;
}

bool sensor::EChem_SWV::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0xffff;
    push(calculateCurrent(pData[i], config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  }

  return true;
}

