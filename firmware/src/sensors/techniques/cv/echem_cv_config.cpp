/**
 * @file echem_cv_config.cpp
 * @brief Cyclic voltammetry (CV) hardware and ramp configuration helpers.
 */

#include "sensors/techniques/cv/echem_cv.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_init_helpers.h"

#include <cmath>

int32_t sensor::EChem_CV::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_SINC3, true, SEQMEMSIZE_2KB,
       AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0},
      &LFOSCFreq);
}

AD5940Err sensor::EChem_CV::setupMeasurement(void) {
  const setupMeasurement::SetupPipeline setup = {
      seq_buffer, SEQ_BUFF_SIZE, config.FifoThresh, {SEQMEMSIZE_2KB, FIFOSRC_SINC3, FIFOSRC_SINC2NOTCH, FIFOSIZE_4KB}};
  AD5940Err error = setup.begin();
  if (error != AD5940ERR_OK) return error;

  // Internal RTIA is opened. Use the configured external RTIA value.
  if (config.LptiaRtiaSel == LPTIARTIA_OPEN) {
    config.RtiaCalValue.Magnitude = config.ExtRtiaVal;
    config.RtiaCalValue.Phase = 0;
  }

  // Calibrate RTIA when using the internal RTIA path.
  if (config.ExtRtia == bFALSE) {
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

  // Generate DAC sequence.
  config.bFirstDACSeq = bTRUE;
  error = generateDACSequence();
  if (error != AD5940ERR_OK) return error;

  setup.triggerInitBlocking(config.InitSeqInfo);

  setup.publishSequencesNoSram({&config.ADCSeqInfo, &config.DACSeqInfo});

  setup.armForWakeup();

  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // Set low-power mode and bandwidth.

  return AD5940ERR_OK;
}

void sensor::EChem_CV::configureRampParameters(void) {
  config.SampleDelayMs = 0.75f * config.PulseWidthMs; // Sample in the last 25% of pulse width.

  const float stepMagnitude = fabsf(config.Estep);
  const bool forwardFirst = (config.Evertex1 > config.Estart);
  config.RampStepMv = forwardFirst ? stepMagnitude : -stepMagnitude;

  // Start one step before/after Estart so the first DAC step lands on Estart.
  const float adjustedStart = config.Estart - config.RampStepMv;
  config.RampStartVolt = -adjustedStart;
  config.RampPeakVolt = -config.Evertex1;

  const float dV1 = fabsf(config.Evertex1 - adjustedStart);
  float dV2;
  float dV3 = 0.0f;

  if (config.Evertex2 == (adjustedStart + config.RampStepMv)) {
    // End one point before returning to Estart.
    dV2 = fabsf(config.Evertex2 - config.Evertex1 + config.RampStepMv);
  } else {
    dV2 = fabsf(config.Evertex2 - config.Evertex1);
    dV3 = fabsf(adjustedStart - config.Evertex2);
  }

  config.StepNumberSeg1 = static_cast<uint32_t>(round(dV1 / stepMagnitude));
  config.StepNumberSeg2 = static_cast<uint32_t>(round(dV2 / stepMagnitude));
  config.StepNumberSeg3 = static_cast<uint32_t>(round(dV3 / stepMagnitude));
  config.StepNumber = config.StepNumberSeg1 + config.StepNumberSeg2 + config.StepNumberSeg3;
}
