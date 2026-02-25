/**
 * @file echem_ocp_config.cpp
 * @brief Open-circuit potential (OCP) hardware and runtime configuration.
 */

#include "sensors/techniques/ocp/echem_ocp.h"
#include "sensors/techniques/setup_measurement_helpers.h"
#include "sensors/techniques/technique_init_helpers.h"

int32_t sensor::EChem_OCP::initAD5940(void) {
  return techniqueInit::initializeAD5940(
      {FIFOSIZE_2KB, FIFOSRC_DFT, true, SEQMEMSIZE_2KB, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ}, &LFOSCFreq);
}

AD5940Err sensor::EChem_OCP::setupMeasurement(void) {
  const setupMeasurement::SetupPipeline setup = {
      seq_buffer, SEQ_BUFF_SIZE, config.FifoThresh, {SEQMEMSIZE_2KB, FIFOSRC_SINC3, FIFOSRC_SINC2NOTCH, FIFOSIZE_4KB}};
  AD5940Err error = setup.begin();
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
