/**
 * @file echem_cv.cpp
 * @brief Cyclic voltammetry (CV) orchestration and lifecycle control.
 */

#include "sensors/techniques/cv/echem_cv.h"

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

sensor::EChem_CV::EChem_CV() {
  memset(&config, 0, sizeof(CVConfig));
  initDefaults();
}

void sensor::EChem_CV::initDefaults(void) {
  config.hasValidParameters = bFALSE; // Flag used to indicate parameters have been set

  /* Step1: configure general parameters */
  config.SeqStartAddr = 0x10;     /* leave 16 commands for LFOSC calibration.  */
  config.MaxSeqLen = 1024 - 0x10; /* 4kB/4 = 1024  */
  config.RcalVal = 10000.0;       /* 10kOhm RCAL */
  config.ADCRefVolt = 1.82;       /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;     /*CV does not need high BW*/
  config.ADCPgaGain = ADCPGA_1P5; /* Gain = 1.5V/V is the factory-calibrated most accurate gain setting. */
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.DataFifoSrc = DATATYPE_SINC2;

  config.LptiaRtiaSel = LPTIARTIA_10K; /* Maximum current decides RTIA value */
  config.LpTiaRf = LPTIARF_OPEN;
  config.LpTiaRl = LPTIARLOAD_SHORT;

  // Some default values for CV -- will be overwritten by loadParameters
  config.RampStartVolt = -1000.0f; /* -1V */
  config.RampPeakVolt = +1000.0f;  /* +1V */
  config.StepNumber = 866;
  config.SampleDelayMs = 1.0f; /* 1 ms */
  config.VzeroStart = 1300.0f;
  config.VzeroPeak = 1300.0f;

  config.ExtRtiaVal = 10000000; // value of external TIA resistor (if used). Update as needed.

  config.bTestFinished = bFALSE;

  config.bFirstDACSeq = bTRUE;

  rampState = RampState::Start; // Initialize the ramp state
  channel = 0;
}


bool sensor::EChem_CV::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating CV parameters...");
  CVParameterPayload params;
  if (!parameterIO::parsePayload("CV", data, len, &params)) return false;

  parameterIO::logField("Processing Interval [s]: ", params.processingInterval);
  parameterIO::logField("Max Current [mA]: ", params.maxCurrent);
  parameterIO::logField("Starting Potential [mV]: ", params.Estart);
  parameterIO::logField("Vertex1 Potential [mV]: ", params.Evertex1);
  parameterIO::logField("Vertex2 Potential [mV]: ", params.Evertex2);
  parameterIO::logField("Step Potential [mV]: ", params.Estep);
  parameterIO::logField("Pulse Width [ms]: ", params.pulseWidth);
  parameterIO::logField("Channel: ", params.channel);

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  // Set the parameters in the configuration structure
  config.Estart = params.Estart;
  config.Evertex1 = params.Evertex1;
  config.Evertex2 = params.Evertex2;
  config.Estep = params.Estep;
  config.PulseWidthMs = params.pulseWidth;
  config.FifoThresh = techniqueLifecycle::deriveFifoThreshold(
      params.processingInterval, params.pulseWidth / 1000.0f, kCVMinFifoThreshold, kCVMaxFifoThreshold);

  channel = params.channel;

  config.hasValidParameters = bTRUE;

  return true;
}


bool sensor::EChem_CV::start() {
  if (config.hasValidParameters != bTRUE) return false; // Parameters have not been set

  clear();                    // Clear the data queue
  power::powerOnAFE(channel); // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();         // Initialize SPI
  if (initAD5940() != 0) {
    cleanupStartFailure(String("CV start failed during AD5940 initialization."));
    return false;
  }
  configureRampParameters();  // Define parameters for the measurement
  const AD5940Err setupError = setupMeasurement();
  if (setupError != AD5940ERR_OK) {
    cleanupStartFailure(String("CV start failed during setupMeasurement. AD5940Err=") + String((int)setupError));
    return false;
  }

  if (sensor::setupMeasurement::wakeupAFE() != AD5940ERR_OK) {
    cleanupStartFailure(String("CV start failed waking AD5940."));
    return false;
  }

  if (rampState == RampState::Stop) {
    cleanupStartFailure(String("CV start failed because ramp state is Stop."));
    return false;
  }

  /* Start it */
  WUPTCfg_Type wupt_cfg;
  String wakeCfgError;
  if (!buildWakeupTimerConfig(wupt_cfg, &wakeCfgError)) {
    cleanupStartFailure(String("CV start failed configuring wakeup timer: ") + wakeCfgError);
    return false;
  }
  AD5940_WUPTCfg(&wupt_cfg);

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();

  return true;
}


bool sensor::EChem_CV::stop() {
  if (!isRunning()) return true;
  if (!techniqueRuntime::stopAndPowerDown()) return false;
  setStopped();
  return true;
}


void sensor::EChem_CV::ISR(void) {
  if (!isRunning()) return;

  std::vector<uint32_t> buf;
  const bool reachedEndSequence = techniqueRuntime::handleInterruptAndDrain(buf, [this]() { generateDACSequence(); });

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
  if (reachedEndSequence) {
    rampState = RampState::Start;
    config.bFirstDACSeq = bTRUE;
    config.bDACCodeInc = bTRUE;
    const bool stopOK = stop();
    updateStatus(stopOK ? TestState::NOT_RUNNING : TestState::ERROR);
  }
}


void sensor::EChem_CV::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}

bool sensor::EChem_CV::validateParameters(const CVParameterPayload& params, String* reason) const {
  if (!validation::requireFinite(
          {params.processingInterval, params.maxCurrent, params.Estart, params.Evertex1, params.Evertex2, params.Estep,
           params.pulseWidth},
          "CV parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.processingInterval, "CV processingInterval must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.pulseWidth, "CV pulseWidth must be > 0.", reason)) return false;
  if (!validation::requireNonZero(params.Estep, "CV Estep must be non-zero.", reason)) return false;
  if (!validation::requireChannelInRange(params.channel, kCVMaxChannel, "CV channel must be in range [0, 3].", reason))
    return false;

  const float minPotential = AD5940_MIN_DAC_OUTPUT - config.VzeroStart;
  const float maxPotential = AD5940_MAX_DAC_OUTPUT - config.VzeroStart;
  const String rangeReason = String("CV voltage parameters are out of range for current Vzero. Valid range [") +
                             String(minPotential) + String(", ") + String(maxPotential) + String("] mV.");
  if (!validation::requireWithinRange(params.Estart, minPotential, maxPotential, rangeReason, reason)) return false;
  if (!validation::requireWithinRange(params.Evertex1, minPotential, maxPotential, rangeReason, reason)) return false;
  if (!validation::requireWithinRange(params.Evertex2, minPotential, maxPotential, rangeReason, reason)) return false;

  return true;
}

bool sensor::EChem_CV::buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const {
  cfg = {};
  cfg.WuptEn = bTRUE;
  cfg.WuptEndSeq = WUPTENDSEQ_D;
  cfg.WuptOrder[0] = SEQID_0;
  cfg.WuptOrder[1] = SEQID_2;
  cfg.WuptOrder[2] = SEQID_1;
  cfg.WuptOrder[3] = SEQID_2;

  uint32_t sampleDelayWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(LFOSCFreq, config.SampleDelayMs, kCVWakeAdjustTicks,
                                                         "CV wakeup timer values are invalid.", reason,
                                                         &sampleDelayWakeTicks))
    return false;
  uint32_t highWidthWakeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromMs(LFOSCFreq, config.PulseWidthMs - config.SampleDelayMs,
                                                         kCVWakeAdjustTicks, "CV wakeup timer values are invalid.",
                                                         reason, &highWidthWakeTicks))
    return false;

  cfg.SeqxSleepTime[SEQID_2] = kCVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_2] = sampleDelayWakeTicks;
  cfg.SeqxSleepTime[SEQID_0] = kCVWakeSleepTicks;
  cfg.SeqxWakeupTime[SEQID_0] = highWidthWakeTicks;
  cfg.SeqxSleepTime[SEQID_1] = cfg.SeqxSleepTime[SEQID_0];
  cfg.SeqxWakeupTime[SEQID_1] = cfg.SeqxWakeupTime[SEQID_0];
  return true;
}

void sensor::EChem_CV::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}


