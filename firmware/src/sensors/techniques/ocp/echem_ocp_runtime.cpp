/**
 * @file echem_ocp_runtime.cpp
 * @brief Open-circuit potential (OCP) runtime data processing.
 */

#include "sensors/techniques/ocp/echem_ocp.h"
#include "sensors/techniques/technique_runtime_helpers.h"

bool sensor::EChem_OCP::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  return techniqueRuntime::forEachAdcCode(pData, numSamples, [this](uint32_t adcCode) {
    push(1000.0f * AD5940_ADCCode2Volt(adcCode, config.ADCPgaGain, config.ADCRefVolt));
  });
}
