/**
 * @file echem_imp_config.cpp
 * @brief Impedance (IMP) hardware and runtime configuration.
 */

#include "sensors/techniques/imp/echem_imp.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_init_helpers.h"

int32_t sensor::EChem_Imp::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_4KB, FIFOSRC_DFT, false, SEQMEMSIZE_2KB, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ}, &LFOSCFreq);
}

AD5940Err sensor::EChem_Imp::setupMeasurement(void) {
  const setupMeasurement::SetupPipeline setup = {
      seq_buffer, SEQ_BUFF_SIZE, config.FifoThresh, {SEQMEMSIZE_2KB, FIFOSRC_DFT, FIFOSRC_DFT, FIFOSIZE_4KB}};
  AD5940Err error = setup.begin();
  if (error != AD5940ERR_OK) return error;

  // Calibrate high-speed RTIA.
  AD5940_CalibrateHSRTIA();

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
  AD5940_WriteReg(REG_AFE_SWMUX, 1 << 3);

  return AD5940ERR_OK;
}

void sensor::EChem_Imp::configureWaveformParameters(void) {
  // Select AFE mode from excitation frequency.
  if (config.SinFreq >= 20000.0)
    config.PwrMod = AFEPWR_HP;
  else
    config.PwrMod = AFEPWR_LP;

  // Convert to peak-to-peak. With EXCITBUFGAIN * HsDacGain == 2, Eac is already effective Vpp.
  config.DacVoltPP = config.Eac;
}
