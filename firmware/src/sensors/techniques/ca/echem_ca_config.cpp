/**
 * @file echem_ca_config.cpp
 * @brief Chronoamperometry (CA) AD5940 initialization and measurement setup.
 */

#include "sensors/techniques/ca/echem_ca.h"
#include "sensors/techniques/technique_init_helpers.h"
#include "sensors/techniques/setup_measurement_helpers.h"

// Initialize AD5940 baseline blocks (clock/FIFO/sequencer/interrupts/GPIO).
int32_t sensor::EChem_CA::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_DFT, true, SEQMEMSIZE_2KB, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ}, &LFOSCFreq);
}

AD5940Err sensor::EChem_CA::setupMeasurement(void) {
  const setupMeasurement::SetupPipeline setup = {
      seq_buffer, SEQ_BUFF_SIZE, config.FifoThresh, {SEQMEMSIZE_2KB, FIFOSRC_SINC3, FIFOSRC_SINC2NOTCH, FIFOSIZE_4KB}};
  AD5940Err error = setup.begin();
  if (error != AD5940ERR_OK) return error;

  // Internal RTIA disabled: use host-configured external RTIA.
  if (config.LptiaRtiaSel == LPTIARTIA_OPEN) {
    config.RtiaCalValue.Magnitude = config.ExtRtiaVal;
  }

  // Calibrate RTIA when using internal RTIA path.
  if (config.ExtRtia == bFALSE) {
    AD5940_CalibrateRTIA(config.AdcClkFreq, config.SysClkFreq, config.LptiaRtiaSel, config.RcalVal,
                         &config.RtiaCalValue);
  }

  // RTIA calibration can leave FIFO/interrupt state dirty. Re-arm runtime FIFO/INT/SEQGEN state afterward.
  error = setup.rearmAfterCalibration();
  if (error != AD5940ERR_OK) return error;

  // Build initialization sequence.
  error = generateInitSequence();
  if (error != AD5940ERR_OK) return error;

  // Build periodic measurement sequence.
  error = generateMeasurementSequence();
  if (error != AD5940ERR_OK) return error;

  setup.triggerInitBlocking(config.InitSeqInfo);

  setup.publishSequenceNoSram(config.MeasureSeqInfo);

  setup.armForWakeup();

  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // Set AFE power mode and bandwidth.

  return AD5940ERR_OK;
}

