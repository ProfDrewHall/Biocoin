/**
 * @file sensor.cpp
 * @brief Shared sensor utility implementations.
 */

#include "sensors/sensor.h"
#include "drivers/ad5940_hal.h"


/* Calculate current in uA */
float sensor::Sensor::calculateCurrent(uint32_t code, uint32_t PGAGain, float VRef, float rTIA) {
  float fCurrent = AD5940_ADCCode2Volt(code, PGAGain, VRef) / rTIA;
  return fCurrent * 1000000; // Convert to uA
}
