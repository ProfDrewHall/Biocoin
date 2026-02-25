/**
 * @file echem_ca_runtime.cpp
 * @brief Chronoamperometry (CA) sample conversion and queueing.
 */

#include "sensors/techniques/ca/echem_ca.h"
#include "sensors/techniques/technique_runtime_helpers.h"


bool sensor::EChem_CA::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  return techniqueRuntime::forEachAdcCode(pData, numSamples, [this](uint32_t adcCode) {
    push(calculateCurrent(adcCode, config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  });
}
