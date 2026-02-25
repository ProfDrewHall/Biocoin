/**
 * @file echem_dpv_config.cpp
 * @brief Differential pulse voltammetry (DPV) hardware and runtime configuration.
 */

#include "sensors/techniques/dpv/echem_dpv.h"
#include "sensors/techniques/lploop_helpers.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_init_helpers.h"

#include "HWConfig/constants.h"

int32_t sensor::EChem_DPV::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_SINC3, true, SEQMEMSIZE_4KB,
       AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0},
      &LFOSCFreq);
}

AD5940Err sensor::EChem_DPV::setupMeasurement(void) {
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
  error = generateADCSequenceHigh();
  if (error != AD5940ERR_OK) return error;
  error = generateADCSequenceLow();
  if (error != AD5940ERR_OK) return error;

  // Generate DAC sequence.
  config.bFirstDACSeq = bTRUE;
  error = generateDACSequence();
  if (error != AD5940ERR_OK) return error;

  // Configure sequence descriptors.
  setup.publishSequencesNoSram({&config.ADC_HighPulse_SeqInfo, &config.ADC_LowPulse_SeqInfo, &config.DACSeqInfo});

  setup.armForWakeup();

  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // Set low-power mode and bandwidth.

  return AD5940ERR_OK;
}

AD5940Err sensor::EChem_DPV::configureLPLoop(void) {
  const int32_t dacData6Bit =
      static_cast<int32_t>((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
  const int32_t dacData12Bit = static_cast<int32_t>(dacData6Bit * 64 + config.RampStartVolt / AD5940_12BIT_DAC_1LSB);
  lpLoop::configureAmperometricLoop(config.ExtRtia, config.LpTiaRf, config.LpTiaRl, config.LptiaRtiaSel, dacData6Bit,
                                     dacData12Bit);

  return AD5940ERR_OK;
}

void sensor::EChem_DPV::configureWaveformParameters() {
  config.bSqrWaveHiLevel = bTRUE;
  rampState = DPVRampState::Start;
  config.bFirstDACSeq = bTRUE;

  config.SampleDelayHighMs = 0.75f * config.PulseWidthMs;
  config.SampleDelayLowMs = 0.75f * (config.PulsePeriodMs - config.PulseWidthMs);

  // Vbias = -Ewe, so RE ramps opposite the desired WE-RE sweep.
  config.RampStartVolt = -config.Estart;
  const float stepAbs = fabsf(config.Estep);
  config.RampPeakVolt = (config.Estop > config.Estart) ? -config.Estop - stepAbs : -config.Estop + stepAbs;
}
