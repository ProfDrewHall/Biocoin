/**
 * @file echem_cv_runtime.cpp
 * @brief Cyclic voltammetry (CV) ramp progression and sample conversion.
 */

#include "sensors/techniques/cv/echem_cv.h"
#include "HWConfig/constants.h"


AD5940Err sensor::EChem_CV::updateRampDACCode(uint32_t* pDACData) {
  if (pDACData == nullptr) return AD5940ERR_PARA;

  // 1) Handle state transitions (only when thresholds are crossed)
  switch (rampState) {
  case RampState::Start: // Estart -> Evertex1
    config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    config.bDACCodeInc = (config.RampPeakVolt > config.RampStartVolt) ? bTRUE : bFALSE;
    rampState = RampState::Vertex1;
    break;

  case RampState::Vertex1: // Evertex1 -> Evertex2
    if (config.CurrStepPos >= config.StepNumberSeg1) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroPeak - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      config.bDACCodeInc = (config.RampPeakVolt > -config.Evertex2) ? bFALSE : bTRUE;
      rampState = RampState::Vertex2;
    }
    break;

  case RampState::Vertex2: // Evertex2 -> Estart
    if (config.CurrStepPos >= (config.StepNumberSeg1 + config.StepNumberSeg2)) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      config.bDACCodeInc = (config.RampStartVolt > -config.Evertex2) ? bTRUE : bFALSE;
      rampState = RampState::Return;
    }
    break;

  case RampState::Return:
    if (config.CurrStepPos >= config.StepNumber) rampState = RampState::Stop;
    break;

  case RampState::Stop:
    break;
  }

  // 2) Advance step position and ramp code
  config.CurrStepPos++;
  config.CurrRampCode += (config.bDACCodeInc ? +config.DACCodePerStep : -config.DACCodePerStep);

  // 3) Compose VZERO (6b) + VBIAS (12b), with saturation
  uint32_t VzeroCode = std::min<uint32_t>(config.CurrVzeroCode, 63); // VzeroCode is 6 bits, max value is 63
  uint32_t VbiasCode = (uint32_t)(VzeroCode * 64 + config.CurrRampCode);
  if (VbiasCode < (VzeroCode * 64)) VbiasCode--;
  if (VbiasCode > 4095) VbiasCode = 4095;

  *pDACData = (VzeroCode << 12) | VbiasCode;
  return AD5940ERR_OK;
}


bool sensor::EChem_CV::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0xffff;
    push(calculateCurrent(pData[i], config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  }

  return true;
}
