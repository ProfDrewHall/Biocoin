/**
 * @file echem_ca.cpp
 * @brief Chronoamperometry (CA) orchestration and lifecycle control.
 */

#include "sensors/techniques/ca/echem_ca.h"

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

sensor::EChem_CA::EChem_CA() {
  memset(&config, 0, sizeof(CAConfig));
  memset(&hostParams, 0, sizeof(CAParameterPayload));
  initDefaults();
}

void sensor::EChem_CA::initDefaults(void) {
  config.hasValidParameters = bFALSE;
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.RcalVal = 10000.0;
  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;
  config.ADCPgaGain = ADCPGA_1P5;
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44;
  config.DataFifoSrc = DATATYPE_SINC2;
  config.LptiaRtiaSel = LPTIARTIA_10K;
  config.LpTiaRf = LPTIARF_OPEN;
  config.LpTiaRl = LPTIARLOAD_SHORT;
  config.ExtRtia = bFALSE;

  config.Vzero = ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 2 + AD5940_MIN_DAC_OUTPUT);

  config.ADCRefVolt = 1.82;
  config.ExtRtiaVal = 10000000;

  channel = 0;
}

bool sensor::EChem_CA::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating CA parameters...");
  CAParameterPayload params;
  if (!parameterIO::parsePayload("CA", data, len, &params)) return false;

  parameterIO::logField("Sampling Interval [s]: ", params.samplingInterval);
  parameterIO::logField("Processing Interval [s]: ", params.processingInterval);
  parameterIO::logField("Max Current [mA]: ", params.maxCurrent);
  parameterIO::logField("Pulse Potential [mV]: ", params.pulsePotential);
  parameterIO::logField("Channel: ", params.channel);

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  hostParams = params;

  config.FifoThresh = techniqueLifecycle::deriveFifoThreshold(params.processingInterval, params.samplingInterval,
                                                              kCAMinFifoThreshold, kCAMaxFifoThreshold);
  config.SamplingIntervalS = params.samplingInterval;
  config.ProcessingIntervalS = params.processingInterval;
  config.SensorBias = params.pulsePotential;
  channel = params.channel;

  config.hasValidParameters = bTRUE;

  return true;
}

bool sensor::EChem_CA::start() {
  if (config.hasValidParameters != bTRUE) return false;

  clear();
  power::powerOnAFE(channel);
  Start_AD5940_SPI();

  if (initAD5940() != 0) {
    cleanupStartFailure(String("CA start failed during AD5940 initialization."));
    return false;
  }

  const AD5940Err setupError = setupMeasurement();
  if (setupError != AD5940ERR_OK) {
    cleanupStartFailure(String("CA start failed during setupMeasurement. AD5940Err=") + String((int)setupError));
    return false;
  }

  if (setupMeasurement::wakeupAFE() != AD5940ERR_OK) {
    cleanupStartFailure(String("CA start failed waking AD5940."));
    return false;
  }

  WUPTCfg_Type wuptCfg = {};
  String wakeupError;
  if (!buildWakeupTimerConfig(wuptCfg, &wakeupError)) {
    cleanupStartFailure(String("CA start failed configuring wakeup timer: ") + wakeupError);
    return false;
  }
  AD5940_WUPTCfg(&wuptCfg);
  AD5940_EnterSleepS();

  Stop_AD5940_SPI();
  setRunning();

  dbgInfo("CA started successfully.");
  return true;
}

bool sensor::EChem_CA::stop() {
  if (!isRunning()) return true;

  // Best-effort final FIFO drain so samples are not lost if an interrupt edge was missed.
  Start_AD5940_SPI();
  if (setupMeasurement::wakeupAFE() == AD5940ERR_OK) {
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    const uint32_t numSamples = AD5940_FIFOGetCnt();
    if (numSamples > 0u) {
      std::vector<uint32_t> buf(numSamples);
      AD5940_FIFORd(buf.data(), numSamples);
      processAndStoreData(buf.data(), numSamples);
    }
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    AD5940_EnterSleepS();
  }
  Stop_AD5940_SPI();

  if (!techniqueRuntime::stopAndPowerDown()) return false;
  setStopped();

  dbgInfo("CA stopped successfully.");
  return true;
}

void sensor::EChem_CA::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}

bool sensor::EChem_CA::buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const {
  cfg.WuptEn = bTRUE;
  cfg.WuptEndSeq = WUPTENDSEQ_A;
  cfg.WuptOrder[0] = SEQID_0;
  cfg.SeqxSleepTime[SEQID_0] = kCAWakeSleepTicks;

  uint32_t wakeupTimeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromSeconds(
          LFOSCFreq, config.SamplingIntervalS, kCAWakeAdjustTicks,
          "sampling interval is too short for wakeup timer configuration", reason, &wakeupTimeTicks))
    return false;

  cfg.SeqxWakeupTime[SEQID_0] = wakeupTimeTicks;
  return true;
}

bool sensor::EChem_CA::validateParameters(const CAParameterPayload& params, String* reason) const {
  if (!validation::requireFinite({params.samplingInterval, params.processingInterval, params.maxCurrent, params.pulsePotential},
                                 "CA parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.samplingInterval, "CA samplingInterval must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.processingInterval, "CA processingInterval must be > 0.", reason))
    return false;
  if (!validation::requireGreaterOrEqual(params.processingInterval, params.samplingInterval,
                                         "Processing interval needs to be more than sampling interval.", reason))
    return false;
  if (!validation::requireChannelInRange(params.channel, kCAMaxChannel, "CA channel must be in range [0, 3].", reason))
    return false;

  const float minBias = AD5940_MIN_DAC_OUTPUT - config.Vzero;
  const float maxBias = AD5940_MAX_DAC_OUTPUT - config.Vzero;
  const String rangeReason = String("Pulse potential is out of range for current Vzero. Valid range [") + String(minBias) +
                             String(", ") + String(maxBias) + String("] mV.");
  return validation::requireWithinRange(params.pulsePotential, minBias, maxBias, rangeReason, reason);
}

void sensor::EChem_CA::ISR(void) {
  if (!isRunning()) return;

  std::vector<uint32_t> buf;
  techniqueRuntime::handleDataFifoThresholdOnly(buf);

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
}

void sensor::EChem_CA::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}
