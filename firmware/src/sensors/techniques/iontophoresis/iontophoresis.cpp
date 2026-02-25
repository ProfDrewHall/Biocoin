/**
 * @file iontophoresis.cpp
 * @brief Iontophoresis stimulation and monitoring task implementation.
 */

#include "sensors/techniques/iontophoresis/iontophoresis.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/core/sensor_manager.h"
#include "sensors/techniques/parameter_io_helpers.h"
#include "sensors/techniques/sensor_validation.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

#include <cmath>

sensor::Iontophoresis::Iontophoresis() {
  memset(&config, 0, sizeof(IontophoresisConfig));
  initDefaults();
  stimulationTaskHandle = nullptr;
}

void sensor::Iontophoresis::initDefaults(void) {
  config.hasValidParameters = bFALSE;
  config.Rsense = 100;
  config.Av_CSA = 20;
}

bool sensor::Iontophoresis::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating Iontophoresis parameters...");
  IontophoresisParameterPayload params;
  if (!parameterIO::parsePayload("iontophoresis", data, len, &params)) return false;

  parameterIO::logField("Sampling Interval [s]: ", params.samplingInterval);
  parameterIO::logField("Stimulation Current [uA]: ", params.stimCurrent);
  parameterIO::logField("Safety Threshold [uA]: ", params.maxCurrent);

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  config.samplingInterval = params.samplingInterval;
  config.stimCurrent = params.stimCurrent;
  config.maxCurrent = params.maxCurrent;
  config.hasValidParameters = bTRUE;

  return true;
}

bool sensor::Iontophoresis::start() {
  if (config.hasValidParameters != bTRUE) return false; // Parameters have not been set

  power::powerOnIontophoresis();
  Start_AD5940_SPI(); // Initialize SPI
  if (initAD5940() != 0) {
    cleanupStartFailure(String("Iontophoresis start failed during AD5940 initialization."));
    return false;
  }
  const AD5940Err configureError = configureDAC();
  if (configureError != AD5940ERR_OK) {
    cleanupStartFailure(String("Iontophoresis start failed during DAC configuration. AD5940Err=") +
                        String((int)configureError));
    return false;
  }

  // Put the AFE to sleep, but leaving the DAC on
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  AD5940_EnterSleepS(); // Enter Hibernate

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power

  startStimulationMonitoringTask(); // Start the monitoring task
  setRunning();
  return true;
}

bool sensor::Iontophoresis::stop() {
  if (!isRunning()) return true;
  Start_AD5940_SPI();
  stopStimulationMonitoringTask();
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
  /* There is chance this operation will fail because sequencer could put AFE back
      to hibernate mode just after waking up. Use STOPSYNC is better. */
  AD5940_WUPTCtrl(bFALSE);
  AD5940_ShutDownS();
  Stop_AD5940_SPI();             // Once the test has started, turn off SPI to reduce power
  power::powerOffPeripherals(); // Shut down the test
  setStopped();
  return true;
}

bool sensor::Iontophoresis::validateParameters(const IontophoresisParameterPayload& params, String* reason) const {
  if (!validation::requireFinite({params.samplingInterval, params.stimCurrent, params.maxCurrent},
                                 "Iontophoresis parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.samplingInterval, "Iontophoresis samplingInterval must be > 0.", reason))
    return false;
  if (!validation::requirePositive(params.maxCurrent, "Iontophoresis maxCurrent must be > 0.", reason)) return false;
  if (!validation::requireNonNegative(params.stimCurrent, "Iontophoresis stimCurrent must be >= 0.", reason))
    return false;
  return validation::requireLessOrEqual(params.stimCurrent, params.maxCurrent,
                                        "Iontophoresis stimCurrent must be <= maxCurrent.", reason);
}

void sensor::Iontophoresis::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}


