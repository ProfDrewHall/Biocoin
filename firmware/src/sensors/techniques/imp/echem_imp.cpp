/**
 * @file echem_imp.cpp
 * @brief Impedance spectroscopy (EIS/IMP) technique implementation.
 */

#include "sensors/techniques/imp/echem_imp.h"

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

sensor::EChem_Imp::EChem_Imp() {
  memset(&config, 0, sizeof(ImpConfig));
  memset(&hostParams, 0, sizeof(IMPParameterPayload));
  initDefaults();
}

void sensor::EChem_Imp::initDefaults(void) {
  config.hasValidParameters = bFALSE;
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0f;
  config.SamplingIntervalS = 30.0f;

  config.RcalVal = 10000.0f;
  config.DswitchSel = SWD_CE0;
  config.PswitchSel = SWP_CE0;
  config.NswitchSel = SWN_AIN1;
  config.TswitchSel = SWT_AIN1;
  config.SenseP = ADCMUXP_AIN3;
  config.SenseN = ADCMUXN_AIN2;

  config.PwrMod = AFEPWR_HP;
  config.AFEBW = AFEBW_250KHZ;
  config.ADCPgaGain = ADCPGA_1P5;
  config.ADCSinc3Osr = ADCSINC3OSR_2;
  config.ADCSinc2Osr = ADCSINC2OSR_22;
  config.HstiaRtiaSel = HSTIARTIA_5K;

  config.CtiaSel = 16;
  config.ExcitBufGain = EXCITBUFGAIN_2;
  config.HsDacGain = HSDACGAIN_1;
  config.HsDacUpdateRate = 7;
  config.DacVoltPP = 800.0f;

  config.DftNum = DFTNUM_8192;
  config.DftSrc = DFTSRC_SINC3;
  config.HanWinEn = bTRUE;

  config.SweepCfg.SweepEn = bFALSE;
  config.SweepCfg.SweepStart = 1000.0f;
  config.SweepCfg.SweepStop = 100000.0;
  config.SweepCfg.SweepPoints = 101;
  config.SweepCfg.SweepLog = bTRUE;
  config.SweepCfg.SweepIndex = 0;

  config.StopRequired = bFALSE;
  config.MeasSeqCycleCount = 0u;
}

bool sensor::EChem_Imp::loadParameters(const uint8_t* data, uint16_t len) {
  dbgInfo("Updating IMP parameters...");
  IMPParameterPayload params;
  if (!parameterIO::parsePayload("IMP", data, len, &params)) return false;

  parameterIO::logField("Sampling Interval [s]: ", params.samplingInterval);
  parameterIO::logField("Processing Interval [s]: ", params.processingInterval);
  parameterIO::logField("Max Current [mA]: ", params.maxCurrent);
  parameterIO::logField("Eac Potential [mV]: ", params.Eac);
  parameterIO::logField("Frequency [Hz]: ", params.frequency);
  parameterIO::logBoolField("4-wire Measurement: ", params.fourWire != 0u, "Enabled", "Disabled");
  parameterIO::logBoolField("AC-coupled Measurement: ", params.acCoupled != 0u, "True", "False");

  String reason;
  if (!validateParameters(params, &reason)) {
    dbgError(reason);
    return false;
  }

  hostParams = params;

  config.FifoThresh =
      techniqueLifecycle::deriveFifoThreshold(params.processingInterval, params.samplingInterval, kIMPMinFifoThreshold,
                                              kIMPMaxFifoThreshold, kIMPFifoGranularity, kIMPFifoGranularity);

  config.SamplingIntervalS = params.samplingInterval;
  config.ProcessingIntervalS = params.processingInterval;

  config.imp4Wire = static_cast<BoolFlag>(params.fourWire);
  config.acCoupled = static_cast<BoolFlag>(params.acCoupled);
  config.Eac = params.Eac;
  config.SinFreq = params.frequency;

  config.hasValidParameters = bTRUE;

  return true;
}

bool sensor::EChem_Imp::start() {
  if (config.hasValidParameters != bTRUE) return false;

  clear();
  power::powerOnAFE(0);
  Start_AD5940_SPI();

  if (initAD5940() != 0) {
    cleanupStartFailure(String("IMP start failed during AD5940 initialization."));
    return false;
  }

  configureWaveformParameters();

  const AD5940Err setupError = setupMeasurement();
  if (setupError != AD5940ERR_OK) {
    cleanupStartFailure(String("IMP start failed during setupMeasurement. AD5940Err=") + String((int)setupError));
    return false;
  }

  if (setupMeasurement::wakeupAFE() != AD5940ERR_OK) {
    cleanupStartFailure(String("IMP start failed waking AD5940."));
    return false;
  }

  WUPTCfg_Type wuptCfg = {};
  String wakeupError;
  if (!buildWakeupTimerConfig(wuptCfg, &wakeupError)) {
    cleanupStartFailure(String("IMP start failed configuring wakeup timer: ") + wakeupError);
    return false;
  }
  AD5940_WUPTCfg(&wuptCfg);
  AD5940_EnterSleepS();
  config.FifoDataCount = 0u;

  Stop_AD5940_SPI();
  setRunning();
  return true;
}

bool sensor::EChem_Imp::stop() {
  if (!isRunning()) return true;
  if (!techniqueRuntime::stopAndPowerDown()) return false;
  setStopped();
  return true;
}

void sensor::EChem_Imp::ISR(void) {
  if (!isRunning()) return;

  std::vector<uint32_t> buf;
  const bool reachedEndSequence =
      techniqueRuntime::handleInterruptAndDrain(buf, []() {}, kIMPFifoGranularity);

  // Advance to the next sweep frequency point when sweep mode is enabled.
  if (config.SweepCfg.SweepEn == bTRUE) {
    config.FreqofData = config.SweepCurrFreq;
    config.SweepCurrFreq = config.SweepNextFreq;
    config.RtiaCurrValue[0] = config.RtiaCalTable[config.SweepCfg.SweepIndex][0];
    config.RtiaCurrValue[1] = config.RtiaCalTable[config.SweepCfg.SweepIndex][1];
    AD5940_SweepNext(&config.SweepCfg, &config.SweepNextFreq);
  }

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
  techniqueLifecycle::stopIfEndSequence(
      reachedEndSequence, "IMP", [this]() { return stop(); },
      [this](bool stopOk) { updateStatus(stopOk ? TestState::NOT_RUNNING : TestState::ERROR); });
}

void sensor::EChem_Imp::printResult(void) {
  const float freq = (config.SweepCfg.SweepEn == bTRUE) ? config.FreqofData : config.SinFreq;

  forEach([freq](const fImpPol_Type& imp) {
    Serial.printf("Freq: %.2f [Hz], Mag: %.5f [Ohm], Phase: %.5f [deg]\n", freq, imp.Magnitude,
                  imp.Phase * 180 / MATH_PI);
  });
}

bool sensor::EChem_Imp::validateParameters(const IMPParameterPayload& params, String* reason) const {
  if (!validation::requireFinite(
          {params.samplingInterval, params.processingInterval, params.maxCurrent, params.Eac, params.frequency},
          "IMP parameters contain non-finite values.", reason))
    return false;
  if (!validation::requirePositive(params.samplingInterval, "IMP samplingInterval must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.processingInterval, "IMP processingInterval must be > 0.", reason))
    return false;
  if (!validation::requireGreaterOrEqual(params.processingInterval, params.samplingInterval,
                                         "Processing interval needs to be more than sampling interval.", reason))
    return false;
  if (!validation::requirePositive(params.frequency, "IMP frequency must be > 0.", reason)) return false;
  if (!validation::requirePositive(params.Eac, "IMP Eac must be > 0.", reason)) return false;
  if (!validation::requireBinaryFlag(params.fourWire, "IMP 4-wire flag must be 0 or 1.", reason)) return false;
  return validation::requireBinaryFlag(params.acCoupled, "IMP AC-coupled flag must be 0 or 1.", reason);
}

bool sensor::EChem_Imp::buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const {
  cfg = {};
  cfg.WuptEn = bTRUE;
  cfg.WuptEndSeq = WUPTENDSEQ_A;
  cfg.WuptOrder[0] = SEQID_0;
  cfg.SeqxSleepTime[SEQID_0] = kIMPWakeSleepTicks;

  uint32_t wakeupTimeTicks = 0u;
  if (!techniqueLifecycle::computeWakeupTimeTicksFromSeconds(
          LFOSCFreq, config.SamplingIntervalS, kIMPWakeAdjustTicks, "IMP sampling interval too short for wakeup timer.",
          reason, &wakeupTimeTicks))
    return false;
  cfg.SeqxWakeupTime[SEQID_0] = wakeupTimeTicks;
  return true;
}

void sensor::EChem_Imp::cleanupStartFailure(const String& reason) const {
  dbgError(reason);
  Stop_AD5940_SPI();
  power::powerOffPeripherals();
}


