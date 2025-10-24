#include "sensors/iontophoresis.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"
#include "util/debug_log.h"

using namespace sensor;

namespace sensor {} // namespace sensor

// Structure for how parameters are passed down from the host
struct IONTOPHORESIS_PARAMETERS {
  float samplingInterval; // [s] How often the ADC samples the current
  float stimCurrent;      // [uA]
  float maxCurrent;       // [uA]
} __attribute__((packed));

Iontophoresis::Iontophoresis() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(IontophoresisConfig_Type));
  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set
  config.Rsense = 100;          // sense resistor value used for the current sense amplifier (ohms)
  config.Av_CSA = 20;           // voltage gain of the current sense amplifier (V/V)

  stimulationTaskHandle = nullptr;
}

bool Iontophoresis::loadParameters(uint8_t* data, uint16_t len) {
  dbgInfo("Updating Iontophoresis parameters...");
  if (len != sizeof(IONTOPHORESIS_PARAMETERS)) { // Check to ensure the size is correct
    dbgError(String("Incorrect payload size! Expected ") + String(sizeof(IONTOPHORESIS_PARAMETERS)) +
             String(" but received ") + String(len));
    return false;
  }

  IONTOPHORESIS_PARAMETERS params;
  memcpy(&params, data, sizeof(params));

  dbgInfo(String("\tSampling Interval [s]: ") + String(params.samplingInterval));
  dbgInfo(String("\tStimulation Current [uA]: ") + String(params.stimCurrent));
  dbgInfo(String("\tSafety Threshold [uA]: ") + String(params.maxCurrent));

  // Bounds/validity checking of parameters

  config.samplingInterval = params.samplingInterval;
  config.stimCurrent = params.stimCurrent;
  config.maxCurrent = params.maxCurrent;
  config.bParaChanged = bTRUE;

  return true;
}

bool Iontophoresis::start() {
  if (config.bParaChanged != bTRUE) return false; // Parameters have not been set

  power::powerOnIontophoresis();
  Start_AD5940_SPI(); // Initialize SPI
  initAD5940();       // Initialize the AD5940
  configureDAC();     // Set the DAC output voltage which controls

  // Put the AFE to sleep, but leaving the DAC on
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  AD5940_EnterSleepS(); // Enter Hibernate

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power

  startStimulationMonitoringTask(); // Start the monitoring task
  setRunning();
  return true;
}

bool Iontophoresis::stop() {
  stopStimulationMonitoringTask();
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
  /* There is chance this operation will fail because sequencer could put AFE back
      to hibernate mode just after waking up. Use STOPSYNC is better. */
  AD5940_WUPTCtrl(bFALSE);
  AD5940_ShutDownS();
  Stop_AD5940_SPI();             // Once the test has started, turn off SPI to reduce power
  power::powerOffPeripherials(); // Shut down the test
  setStopped();
  return true;
}

/* Initialize AD5940 basic blocks like clock */
int32_t Iontophoresis::initAD5940(void) {
  AD5940_HWReset();                                // Hardware reset
  AD5940_Initialize();                             // Platform configuration
  AD5940_ConfigureClock();                         // Step 1 - Configure clock
  AD5940_ConfigureFIFO(FIFOSIZE_2KB, FIFOSRC_DFT); // Step 2 - Configure FIFO and Sequencer
  AD5940_ConfigureSequencer(SEQMEMSIZE_2KB);
  AD5940_ConfigureInterrupts(AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ); // Step 3 - Configure interrupt controller
  AD5940_ConfigureGPIO();                                                  // Step 4 - Reconfigure GPIO
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);                                     // Enable AFE to enter sleep mode
  AD5940_MeasureLFOSC(&LFOSCFreq);                                         // Measure the LFOSC frequency

  return 0;
}

AD5940Err Iontophoresis::configureDAC(void) {
  float voltage =
      config.stimCurrent * config.Rsense * config.Av_CSA / 1000; // Convert desired current into a DAC voltage in [mV]

  AD5940_ConfigureAFEReferences(true, true, false, false);
  LPLoopCfg_Type lp_cfg = {0};
  lp_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lp_cfg.LpDacCfg.DacData12Bit = (voltage - AD5940_MIN_DAC_OUTPUT) / AD5940_12BIT_DAC_1LSB;
  lp_cfg.LpDacCfg.DacData6Bit = 0;
  lp_cfg.LpDacCfg.DataRst = bFALSE;
  lp_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2PIN; // | LPDACSW_VBIAS2LPPA;
  lp_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lp_cfg);

  return AD5940ERR_OK;
}

// Current monitoring
void Iontophoresis::startStimulationMonitoringTask() {
  if (stimulationTaskHandle == nullptr) {
    xTaskCreate(stimulationTask,               // Task function
                "Stimulation Current Monitor", // Task name
                512,                           // Stack size (words)
                this,                          // Task parameters
                0,                             // Priority
                &stimulationTaskHandle         // Save handle
    );
  }
}

void Iontophoresis::stopStimulationMonitoringTask() {
  if (stimulationTaskHandle != nullptr) {
    vTaskDelete(stimulationTaskHandle);
    stimulationTaskHandle = nullptr;
  }
}

void Iontophoresis::stimulationTask(void* pvParameters) {
  auto* self = static_cast<Iontophoresis*>(pvParameters);
  if (!self) {
    dbgError("stimulationTask received nullptr parameters!");
    vTaskDelete(nullptr);
  }

  auto& config = self->config;

  while (true) {
    // The ADC is enabled inside the analogRead() function
    // NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;        // Turn on the ADC
    power::reconnectInputGPIO(PIN_CURRENT_SENSE_OUT, power::PullConfig::Disabled); // Turn back on the pin

    // Configure the ADC
    analogReference(AR_INTERNAL_2_4);                        // Set the analog reference (default = 3.6V)
    analogSampleTime(SAADC_CH_CONFIG_TACQ_40us);             // Set the ADC Sample and Hold Acquisition time
    analogOversampling(SAADC_OVERSAMPLE_OVERSAMPLE_Over16x); // Set the ADC to Oversample and perform sample averaging:
                                                             // averages 2^OVERSAMPLE samples per reading
    analogCalibrateOffset();
    analogReadResolution(14); // Set the resolution, can be 8, 10, 12 or 14

    // Get the raw ADC value and average the readings
    uint32_t total = 0;
    for (uint8_t i = 0; i < kNumADCSamplesToAverage; i++)
      total += analogRead(PIN_CURRENT_SENSE_OUT);

    // Revert the ADC to default settings
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;  // Turn off the ADC
    power::disconnectInputGPIO(PIN_CURRENT_SENSE_OUT); // Float the pin to save power

    // Calculate the current [uA]
    float avgADCReading = static_cast<float>(total) / kNumADCSamplesToAverage; // Average the reading
    float current = avgADCReading * kmVperLSB / config.Rsense / config.Av_CSA * 1000;

    dbgInfo(String("Measured current (uA):") + current);

    // check if current exceeds maximum allowed value for safety reasons
    if (current > config.maxCurrent) {
      self->stop();
      vTaskDelete(nullptr); // delete the task
      self->stimulationTaskHandle = nullptr;
      updateStatus(TestState::CURRENT_LIMIT_EXCEEDED);
      dbgError("Current threshold exceeded! Shutting down iontophoresis\n");
    }

    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(config.samplingInterval * 1000.0f)));
  }
}
