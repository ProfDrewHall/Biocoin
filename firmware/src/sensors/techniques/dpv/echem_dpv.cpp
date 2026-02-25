/**
 * @file echem_dpv.cpp
 * @brief Differential pulse voltammetry (DPV) technique implementation.
 */

#include "sensors/techniques/dpv/echem_dpv.h"

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

sensor::EChem_DPV::EChem_DPV() {
  memset(&config, 0, sizeof(DPVConfig));
  initDefaults();
}

void sensor::EChem_DPV::initDefaults(void) {
  config.hasValidParameters = bFALSE; // Flag used to indicate parameters have been set

  config.SeqStartAddr = 0x10;     /* leave 16 commands for LFOSC calibration.  */
  config.MaxSeqLen = 1024 - 0x10; /* 4kB/4 = 1024  */
  config.RcalVal = 10000.0;       /* 10kOhm RCAL */
  config.ADCRefVolt = 1.82;       /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;     /*DPV does not need high BW*/
  config.ADCPgaGain = ADCPGA_1P5; /* Gain = 1.5V/V is the factory-calibrated most accurate gain setting. */
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.DataFifoSrc = DATATYPE_SINC2;

  config.LptiaRtiaSel = LPTIARTIA_10K; /* Maximum current decides RTIA value */
  config.LpTiaRf = LPTIARF_OPEN;
  config.LpTiaRl = LPTIARLOAD_SHORT;

  config.VzeroStart = 1300.0f; /* 1.3V */
  config.VzeroPeak = 1300.0f;  /* 1.3V */

  config.ExtRtiaVal = 10000000; // value of external TIA resistor (if used). Update as needed.
  rampState = DPVRampState::Start;
  channel = 0;
}

bool sensor::EChem_DPV::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating DPV parameters...");
  DPVParameterPayload params;
  if (!parameterIO::parsePayload("DPV", data, len, &params)) return false;

  parameterIO::logField("Processing Interval [s]: ", params.processingInterval);
  parameterIO::logField("Max Current [mA]: ", params.maxCurrent);
  parameterIO::logField("Starting Potential [mV]: ", params.Estart);
  parameterIO::logField("Stop Potential [mV]: ", params.Estop);
  parameterIO::logField("Pulse Potential [mV]: ", params.Epulse);
  parameterIO::logField("Step Potential [mV]: ", params.Estep);
  parameterIO::logField("Pulse Width [ms]: ", params.pulseWidth);
  parameterIO::logField("Pulse Period [ms]: ", params.pulsePeriod);
  parameterIO::logField("Channel: ", params.channel);

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  // Set the parameters in the configuration structure
  config.Estart = params.Estart;
  config.Estop = params.Estop;
  config.Epulse = params.Epulse;
  config.Estep = params.Estep;
  config.PulseWidthMs = params.pulseWidth;
  config.PulsePeriodMs = params.pulsePeriod;
  config.FifoThresh = techniqueLifecycle::deriveFifoThreshold(
      params.processingInterval, params.pulseWidth / 1000.0f, kDPVMinFifoThreshold, kDPVMaxFifoThreshold);
  channel = params.channel;
  config.hasValidParameters = bTRUE;

  return true;
}

bool sensor::EChem_DPV::start() {
  if (config.hasValidParameters != bTRUE) return false; // Parameters have not been set

  clear();                       // Clear the data queue
  power::powerOnAFE(channel);    // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();            // Initialize SPI
  if (initAD5940() != 0) {
    cleanupStartFailure(String("DPV start failed during AD5940 initialization."));
    return false;
  }
  configureWaveformParameters(); // Define parameters for the measurement
  const AD5940Err setupError = setupMeasurement();
  if (setupError != AD5940ERR_OK) {
    cleanupStartFailure(String("DPV start failed during setupMeasurement. AD5940Err=") + String((int)setupError));
    return false;
  }

  if (sensor::setupMeasurement::wakeupAFE() != AD5940ERR_OK) {
    cleanupStartFailure(String("DPV start failed waking AD5940."));
    return false;
  }

  /* Start it */
  WUPTCfg_Type wupt_cfg;
  String wakeCfgError;
  if (!buildWakeupTimerConfig(wupt_cfg, &wakeCfgError)) {
    cleanupStartFailure(String("DPV start failed configuring wakeup timer: ") + wakeCfgError);
    return false;
  }
  AD5940_WUPTCfg(&wupt_cfg);

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();
  return true;
}

bool sensor::EChem_DPV::stop() {
  if (!isRunning()) return true;
  if (!techniqueRuntime::stopAndPowerDown()) return false;
  setStopped();
  rampState = DPVRampState::Stop;
  return true;
}

/* Initialize AD5940 basic blocks like clock */
void sensor::EChem_DPV::ISR(void) {
  if (!isRunning()) return;

  std::vector<uint32_t> buf;
  const bool reachedEndSequence = techniqueRuntime::handleInterruptAndDrain(buf, [this]() { generateDACSequence(); });

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
  if (reachedEndSequence) {
    const bool stopOK = stop();
    updateStatus(stopOK ? TestState::NOT_RUNNING : TestState::ERROR);
  }
}

// Calculate DAC code step by step
void sensor::EChem_DPV::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}

bool sensor::EChem_DPV::validateParameters(const DPVParameterPayload& params, String* reason) const {
  if (!validation::requireFinite({params.processingInterval, params.maxCurrent, params.Estart, params.Estop, params.Epulse,
                                  params.Estep, params.pulseWidth, params.pulsePeriod},
                                 "DPV parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.processingInterval, "DPV processingInterval must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.pulseWidth, "DPV pulseWidth must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.pulsePeriod, "DPV pulsePeriod must be > 0.", reason)) return false;
  if (params.pulsePeriod < params.pulseWidth) {
    if (reason) *reason = "DPV pulsePeriod must be >= pulseWidth.";
    return false;
  }
  if (!validation::requireNonZero(params.Estep, "DPV Estep must be non-zero.", reason)) return false;
  if (!validation::requireChannelInRange(params.channel, kDPVMaxChannel, "DPV channel must be in range [0, 3].", reason))
    return false;

  const float minPotential = AD5940_MIN_DAC_OUTPUT - config.VzeroStart;
  const float maxPotential = AD5940_MAX_DAC_OUTPUT - config.VzeroStart;
  const String rangeReason = String("DPV voltage parameters are out of range for current Vzero. Valid range [") +
                             String(minPotential) + String(", ") + String(maxPotential) + String("] mV.");
  if (!validation::requireWithinRange(params.Estart, minPotential, maxPotential, rangeReason, reason)) return false;
  if (!validation::requireWithinRange(params.Estop, minPotential, maxPotential, rangeReason, reason)) return false;
  if (!validation::requireWithinRange(params.Epulse, minPotential, maxPotential, rangeReason, reason)) return false;

  return true;
}

bool sensor::EChem_DPV::buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const {
  cfg = {};
  cfg.WuptEn = bTRUE;
  cfg.WuptEndSeq = WUPTENDSEQ_D;
  cfg.WuptOrder[0] = SEQID_0;
  cfg.WuptOrder[1] = SEQID_2;
  cfg.WuptOrder[2] = SEQID_1;
  cfg.WuptOrder[3] = SEQID_3;

  uint32_t lowDelayWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(LFOSCFreq, config.SampleDelayLowMs, kDPVWakeAdjustTicks,
                                                         "DPV wakeup timer values are invalid.", reason,
                                                         &lowDelayWakeTicks))
    return false;
  uint32_t highDelayWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(LFOSCFreq, config.SampleDelayHighMs, kDPVWakeAdjustTicks,
                                                         "DPV wakeup timer values are invalid.", reason,
                                                         &highDelayWakeTicks))
    return false;
  uint32_t lowWidthWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(
          LFOSCFreq, config.PulsePeriodMs - config.PulseWidthMs - config.SampleDelayLowMs, kDPVWakeAdjustTicks,
          "DPV wakeup timer values are invalid.", reason, &lowWidthWakeTicks))
    return false;
  uint32_t highWidthWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(LFOSCFreq, config.PulseWidthMs - config.SampleDelayHighMs,
                                                         kDPVWakeAdjustTicks, "DPV wakeup timer values are invalid.",
                                                         reason, &highWidthWakeTicks))
    return false;

  cfg.SeqxSleepTime[SEQID_2] = kDPVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_2] = lowDelayWakeTicks;
  cfg.SeqxSleepTime[SEQID_3] = kDPVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_3] = highDelayWakeTicks;
  cfg.SeqxSleepTime[SEQID_1] = kDPVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_1] = lowWidthWakeTicks;
  cfg.SeqxSleepTime[SEQID_0] = kDPVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_0] = highWidthWakeTicks;
  return true;
}

void sensor::EChem_DPV::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}


