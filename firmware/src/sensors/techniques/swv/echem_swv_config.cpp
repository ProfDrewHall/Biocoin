/**
 * @file echem_swv_config.cpp
 * @brief Square-wave voltammetry (SWV) hardware and runtime configuration.
 */

#include "sensors/techniques/swv/echem_swv.h"
#include "sensors/techniques/lploop_helpers.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_init_helpers.h"

#include "HWConfig/constants.h"

int32_t sensor::EChem_SWV::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_SINC3, true, SEQMEMSIZE_4KB,
       AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0},
      &LFOSCFreq);
}

AD5940Err sensor::EChem_SWV::setupMeasurement(void) {
  const setupMeasurement::SetupPipeline setup = {
      seq_buffer, SEQ_BUFF_SIZE, config.FifoThresh, {SEQMEMSIZE_4KB, FIFOSRC_SINC3, FIFOSRC_SINC2NOTCH, FIFOSIZE_2KB}};
  AD5940Err error = setup.begin();
  if (error != AD5940ERR_OK) return error;

  // Internal RTIA is opened. Use the configured external RTIA value.
  if (config.LptiaRtiaSel == LPTIARTIA_OPEN) {
    config.RtiaCalValue.Magnitude = config.ExtRtiaVal;
    config.RtiaCalValue.Phase = 0;
  } else {
    AD5940_CalibrateRTIA(config.AdcClkFreq, config.SysClkFreq, config.LptiaRtiaSel, config.RcalVal,
                         &config.RtiaCalValue);
  }

  error = setup.rearmAfterCalibration();
  if (error != AD5940ERR_OK) return error;

  // Generate initialization/measurement sequences.
  error = generateInitSequence();
  if (error != AD5940ERR_OK) return error;
  error = generateADCSequence();
  if (error != AD5940ERR_OK) return error;

  config.bFirstDACSeq = bTRUE;
  error = generateDACSequence();
  if (error != AD5940ERR_OK) return error;

  setup.triggerInitBlocking(config.InitSeqInfo);

  setup.publishSequencesNoSram({&config.ADCSeqInfo, &config.DACSeqInfo});

  setup.armForWakeup();
  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // Set low-power mode and bandwidth.
  return AD5940ERR_OK;
}

AD5940Err sensor::EChem_SWV::configureLPLoop(void) {
  // Just initial DAC settings before SWV truly begins.
  const float dir = (config.Estop > config.Estart) ? 1.0f : -1.0f;
  const int32_t dacData6Bit =
      static_cast<int32_t>((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
  const int32_t dacData12Bit = static_cast<int32_t>(
      dacData6Bit * 64 + (config.RampStartVolt + dir * config.RampAmplitudeMv - dir * config.Estep) /
                             AD5940_12BIT_DAC_1LSB);
  lpLoop::configureAmperometricLoop(config.ExtRtia, config.LpTiaRf, config.LpTiaRl, config.LptiaRtiaSel, dacData6Bit,
                                     dacData12Bit);

  return AD5940ERR_OK;
}

void sensor::EChem_SWV::configureWaveformParameters() {
  config.bSqrWaveHiLevel = bTRUE;
  rampState = SWVRampState::Start;
  config.bFirstDACSeq = bTRUE;

  config.SampleDelayMs = 0.75f * config.PulseWidthMs;
  config.RampAmplitudeMv = 2.0f * config.EAmplitude;

  // Vbias = -Ewe, so RE ramps opposite the desired WE-RE sweep.
  const float dir = (config.Estop > config.Estart) ? 1.0f : -1.0f;
  config.RampStartVolt = -config.Estart + dir * (-1.0f * config.RampAmplitudeMv) + dir * config.Estep;
  config.RampPeakVolt = -config.Estop + dir * (-1.0f * config.RampAmplitudeMv);
}
