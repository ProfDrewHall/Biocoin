#include "sensors/Sensor.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "util/debug_log.h"


/* Calculate current in uA */
float sensor::Sensor::calculateCurrent(uint32_t code, uint32_t PGAGain, float VRef, float rTIA) {
  float fCurrent = AD5940_ADCCode2Volt(code, PGAGain, VRef) / rTIA;
  return fCurrent * 1000000; // Convert to uA
}
