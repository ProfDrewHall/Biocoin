/**
 * @file echem_temp.cpp
 * @brief Temperature measurement technique implementation.
 */

#include "sensors/techniques/temp/echem_temp.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/core/sensor_manager.h"
#include "sensors/techniques/parameter_io_helpers.h"
#include "sensors/techniques/sensor_validation.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_lifecycle_helpers.h"
#include "sensors/techniques/technique_runtime_helpers.h"
#include "util/debug_log.h"

#include <cmath>

sensor::EChem_Temp::EChem_Temp() {
  memset(&config, 0, sizeof(TempConfig));
  memset(&hostParams, 0, sizeof(TempParameterPayload));
  initDefaults();
}

void sensor::EChem_Temp::initDefaults(void) {
  config.hasValidParameters = bFALSE;
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;
  config.ADCPgaGain = ADCPGA_1P5;
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44;
  config.DataFifoSrc = DATATYPE_SINC2;
  config.ADCRefVolt = 1.82;
}

bool sensor::EChem_Temp::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating TEMP parameters...");
  TempParameterPayload params;
  if (!parameterIO::parsePayload("TEMP", data, len, &params)) return false;

  parameterIO::logField("Sampling Interval [s]: ", params.samplingInterval);
  parameterIO::logField("Processing Interval [s]: ", params.processingInterval);
  parameterIO::logField("Channel: ", params.channel);

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  hostParams = params;

  config.FifoThresh = techniqueLifecycle::deriveFifoThreshold(params.processingInterval, params.samplingInterval,
                                                              kTempMinFifoThreshold, kTempMaxFifoThreshold);
  config.SamplingIntervalS = params.samplingInterval;
  config.ProcessingIntervalS = params.processingInterval;
  channel = params.channel;

  config.hasValidParameters = bTRUE;

  return true;
}

bool sensor::EChem_Temp::start() {
  if (config.hasValidParameters != bTRUE) return false;

  clear();
  power::powerOnTempSensor();
  Start_AD5940_SPI();

  if (initAD5940() != 0) {
    cleanupStartFailure(String("TEMP start failed during AD5940 initialization."));
    return false;
  }

  const AD5940Err setupError = setupMeasurement();
  if (setupError != AD5940ERR_OK) {
    cleanupStartFailure(String("TEMP start failed during setupMeasurement. AD5940Err=") + String((int)setupError));
    return false;
  }

  if (setupMeasurement::wakeupAFE() != AD5940ERR_OK) {
    cleanupStartFailure(String("TEMP start failed waking AD5940."));
    return false;
  }

  WUPTCfg_Type wuptCfg = {};
  String wakeupError;
  if (!buildWakeupTimerConfig(wuptCfg, &wakeupError)) {
    cleanupStartFailure(String("TEMP start failed configuring wakeup timer: ") + wakeupError);
    return false;
  }
  AD5940_WUPTCfg(&wuptCfg);
  AD5940_EnterSleepS();

  Stop_AD5940_SPI();
  setRunning();
  return true;
}

bool sensor::EChem_Temp::stop() {
  if (!isRunning()) return true;
  if (!techniqueRuntime::stopAndPowerDown()) return false;
  setStopped();
  return true;
}

void sensor::EChem_Temp::ISR(void) {
  if (!isRunning()) return;

  std::vector<uint32_t> buf;
  const bool reachedEndSequence = techniqueRuntime::handleInterruptAndDrain(buf, []() {});

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
  techniqueLifecycle::stopIfEndSequence(
      reachedEndSequence, "TEMP", [this]() { return stop(); },
      [this](bool stopOk) { updateStatus(stopOk ? TestState::NOT_RUNNING : TestState::ERROR); });
}

void sensor::EChem_Temp::printResult(void) {
  forEach([](const float& v_mV) { Serial.printf("Voltage = %.5f (mV)\n", v_mV); });
}

bool sensor::EChem_Temp::validateParameters(const TempParameterPayload& params, String* reason) const {
  if (!validation::requireFinite({params.samplingInterval, params.processingInterval},
                                 "TEMP parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.samplingInterval, "TEMP samplingInterval must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.processingInterval, "TEMP processingInterval must be > 0.", reason))
    return false;
  if (!validation::requireGreaterOrEqual(params.processingInterval, params.samplingInterval,
                                         "Processing interval needs to be more than sampling interval.", reason))
    return false;
  return validation::requireOneOf<uint8_t>(params.channel, {kTempChannelVafe4, kTempChannelVafe1, kTempChannelAin2},
                                           "Invalid TEMP channel.", reason);
}

bool sensor::EChem_Temp::buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const {
  cfg = {};
  cfg.WuptEn = bTRUE;
  cfg.WuptEndSeq = WUPTENDSEQ_A;
  cfg.WuptOrder[0] = SEQID_0;
  cfg.SeqxSleepTime[SEQID_0] = kTempWakeSleepTicks;

  uint32_t wakeupTimeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromSeconds(
          LFOSCFreq, config.SamplingIntervalS, kTempWakeAdjustTicks,
          "TEMP sampling interval too short for wakeup timer.", reason, &wakeupTimeTicks))
    return false;
  cfg.SeqxWakeupTime[SEQID_0] = wakeupTimeTicks;
  return true;
}

void sensor::EChem_Temp::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}


