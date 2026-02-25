/**
 * @file echem_imp_runtime.cpp
 * @brief Impedance (IMP) runtime register/data processing.
 */

#include "sensors/techniques/imp/echem_imp.h"

AD5940Err sensor::EChem_Imp::updateRegisters(void) {
  if (config.NumOfData > 0) {
    config.FifoDataCount += getNumBytesAvailable() / 4;
    if (config.FifoDataCount >= static_cast<uint32_t>(config.NumOfData)) {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if (config.StopRequired == bTRUE) {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if (config.SweepCfg.SweepEn) AD5940_WGFreqCtrlS(config.SweepNextFreq, config.SysClkFreq);
  return AD5940ERR_OK;
}

bool sensor::EChem_Imp::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  if (!pData || (numSamples % 4u) != 0u) return false;

  // Convert packed 18-bit two's complement DFT words to int32.
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0x3ffff; // TODO: optionally validate ECC bits.
    if (pData[i] & (1 << 17)) pData[i] |= 0xfffc0000;
  }

  // Reinterpret as interleaved I/Q pairs.
  const iImpCar_Type* impData = reinterpret_cast<const iImpCar_Type*>(pData);

  for (uint32_t i = 0; i < numSamples / 4; i++) {
    // Each DFT result contributes current and voltage complex values.
    const iImpCar_Type& curr = impData[2 * i + 0];
    const iImpCar_Type& volt = impData[2 * i + 1];

    // Compute magnitude/phase for voltage and current.
    const float vm = std::hypot(static_cast<float>(volt.Real), static_cast<float>(volt.Image));
    const float vp = std::atan2(-static_cast<float>(volt.Image), static_cast<float>(volt.Real));

    const float im = std::hypot(static_cast<float>(curr.Real), static_cast<float>(curr.Image));
    const float ip = std::atan2(-static_cast<float>(curr.Image), static_cast<float>(curr.Real));

    // Refer measurements to calibrated RTIA values.
    fImpPol_Type imp;
    imp.Magnitude = vm / im * config.RtiaCurrValue[0];
    imp.Phase = vp - ip + config.RtiaCurrValue[1];
    push(imp);
  }

  return true;
}
