/**
 * @file echem_temp_runtime.cpp
 * @brief Temperature technique runtime data processing.
 */

#include "sensors/techniques/temp/echem_temp.h"
#include "sensors/techniques/technique_runtime_helpers.h"

bool sensor::EChem_Temp::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  return techniqueRuntime::forEachAdcCode(pData, numSamples, [this](uint32_t adcCode) {
    push(1000.0f * AD5940_ADCCode2Volt(adcCode, config.ADCPgaGain, config.ADCRefVolt) + 1110.0f);
  });
}
