/**
 * @file echem_ca.cpp
 * @brief Chronoamperometry (CA) technique implementation.
 */

#include "sensors/echem_ca.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "util/debug_log.h"


// Structure for how parameters are passed down from the host
struct CA_PARAMETERS {
  float samplingInterval;   // [s] How often the ADC samples by controlling the sleep time between sequences
  float processingInterval; // [s] How often the interrupt triggers and thus how often the MCU reads/processes the data
  float maxCurrent;
  float pulsePotential; // [mV] This is the pulse amplitude
  uint8_t channel;      // Channel (0-3) to select
} __attribute__((packed));

sensor::EChem_CA::EChem_CA() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(CAConfig_Type));
  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.RcalVal = 10000.0; // 10kOhm on Biocoin
  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;     /*CA does not need high BW*/
  config.ADCPgaGain = ADCPGA_1P5; /*Gain = 1.5V/V is the factory calibrated most accurate gain setting*/
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.DataFifoSrc = DATATYPE_SINC2; /* Data type must be SINC2 for chrono-amperometric measurement*/
  config.LptiaRtiaSel = LPTIARTIA_10K;
  config.LpTiaRf = LPTIARF_OPEN;
  config.LpTiaRl = LPTIARLOAD_SHORT;
  config.ExtRtia = bFALSE; // false = not using external TIA resistor, true = using it

  // LPDAC Configure, set to midscale values for now (1300mV)
  config.Vzero = ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 2 + AD5940_MIN_DAC_OUTPUT); // midscale

  config.ADCRefVolt = 1.82;     /* Measure voltage on ADCRefVolt pin and enter here*/
  config.ExtRtiaVal = 10000000; // value of external TIA resistor (if used). Update as needed.

  channel = 0;
}

bool sensor::EChem_CA::loadParameters(uint8_t* data, uint16_t len) {
  dbgInfo("Updating CA parameters...");
  if (len != sizeof(CA_PARAMETERS)) { // Check to ensure the size is correct
    dbgError(String("Incorrect payload size! Expected ") + String(sizeof(CA_PARAMETERS)) + String(" but received ") +
              String(len));
    return false;
  }

  CA_PARAMETERS params;
  memcpy(&params, data, sizeof(params));

  dbgInfo(String("\tSampling Interval [s]: ") + String(params.samplingInterval));
  dbgInfo(String("\tProcessing Interval [s]: ") + String(params.processingInterval));
  dbgInfo(String("\tMax Current [mA]: ") + String(params.maxCurrent));
  dbgInfo(String("\tPulse Potential [mV]: ") + String(params.pulsePotential));
  dbgInfo(String("\tChannel: ") + String(params.channel));

  // Bounds/validity checking of parameters
  if (params.processingInterval < params.samplingInterval) {
    dbgError("Processing interval needs to be more than sampling interval.");
    return false;
  }

  // The threshold to set when the interrupt triggers. Number of samples required to reach desired processing time is
  // processing time divided by sampling interval
  config.FifoThresh = (uint32_t)(params.processingInterval / params.samplingInterval);
  config.SamplingInterval = params.samplingInterval;
  config.SensorBias = params.pulsePotential; /* Sensor bias voltage (VWE - VRE) [mV]*/
  channel = params.channel;

  // Still need to use max_current to calculate the gain resistor
  // config.LptiaRtiaSel = LPTIARTIA_10K;		// this sets the current range for the experiment

  config.bParaChanged = bTRUE;

  return true;
}

bool sensor::EChem_CA::start() {
  if (config.bParaChanged != bTRUE) return false; // Parameters have not been set

  clear();                    // Clear the data queue
  power::powerOnAFE(channel); // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();         // Initialize SPI
  initAD5940();               // Initialize the AD5940
  setupMeasurement();         // Initialize measurement sequence

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */

  // Configure Wakeup Timer to trigger above sequence periodically to measure data.
  WUPTCfg_Type wupt_cfg;
  wupt_cfg.WuptEn = bTRUE;
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.SeqxSleepTime[SEQID_0] =
      4 - 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(LFOSCFreq * config.SamplingInterval) - 4 - 1;
  AD5940_WUPTCfg(&wupt_cfg); // will enable Wakeup timer, measurement begins here
  AD5940_EnterSleepS(); // Enter Hibernate now otherwise it won't start sleeping until after the first interrupt period

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();
  return true;
}

bool sensor::EChem_CA::stop() {
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
int32_t sensor::EChem_CA::initAD5940(void) {
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


AD5940Err sensor::EChem_CA::setupMeasurement(void) {
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB; /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  if (config.LptiaRtiaSel == LPTIARTIA_OPEN) { /* Internal RTIA is opened. User wants to use external RTIA resistor */
    config.RtiaCalValue.Magnitude = config.ExtRtiaVal;
  }
  /* Do RTIA calibration */
  if (config.ExtRtia == bFALSE) {
    AD5940_CalibrateRTIA(config.AdcClkFreq, config.SysClkFreq, config.LptiaRtiaSel, config.RcalVal,
                         &config.RtiaCalValue);
  }

  /* Now Reconfigure FIFO after Rtia cal for CA measurements*/
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = config.FifoThresh;
  AD5940_FIFOCfg(&fifo_cfg);
  /* Clear interrupts*/
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Generate sequences */
  if (seq_buffer == 0) return AD5940ERR_PARA;
  if (SEQ_BUFF_SIZE == 0) return AD5940ERR_PARA;
  AD5940_SEQGenInit(seq_buffer, SEQ_BUFF_SIZE);

  /* Generate initialize sequence */
  error = generateInitSequence(); /* Application initialization sequence using either MCU or sequencer */
  if (error != AD5940ERR_OK) return error;

  /* Generate measurement sequence */
  error = generateMeasSequence();
  if (error != AD5940ERR_OK) return error;

  /* Initialize sequences */
  config.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg); /* Enable sequencer */
  AD5940_SEQMmrTrig(config.InitSeqInfo.SeqId);
  while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE)
    ;
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

  /* Measurement sequence  */
  config.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.MeasureSeqInfo);

  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg); /* Enable sequencer, and wait for trigger */

  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); // set to low power mode with desired BW

  return AD5940ERR_OK;
}

/* Generate init sequence for CA. This runs only one time. */
AD5940Err sensor::EChem_CA::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator here
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all to disable state

  AD5940_ConfigureAFEReferences(true, true, true, true);
  AD5940_ConfigureLPLoop(config.Vzero, config.SensorBias, config.LpTiaRf, config.LpTiaRl, config.ExtRtia,
                         config.LptiaRtiaSel);
  AD5940_ConfigureDSP(ADCMUXN_LPTIA0_N, ADCMUXP_LPTIA0_P, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

  HSLoopCfg_Type hs_loop = {0};
  AD5940_HSLoopCfgS(&hs_loop);
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0);

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we
                                      only want it to run one time. */

  /* Stop here */
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen); // create sequence
  if (error == AD5940ERR_OK) {
    config.InitSeqInfo.SeqId = SEQID_1;
    config.InitSeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.InitSeqInfo.pSeqCmd = pSeqCmd;
    config.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(config.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

/* Generate measurement sequence for CA. This runs indefinitely until test is ended. */
AD5940Err sensor::EChem_CA::generateMeasSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = config.DataFifoSrc;
  clks_cal.DataCount = 1; /* Sample one data when wakeup */
  clks_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  // generate sequence to measure data
  AD5940_SEQGenCtrl(bTRUE); // from now on, record all register operations rather than write them to AD5940 through SPI.
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE); // turn on ADC and sinc2 power
  AD5940_SEQGenInsert(
      SEQ_WAIT(16 * 250)); /* Time for reference settling(if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);                                        /* Start ADC convert*/
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));                                       /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC */
  AD5940_SEQGenInsert(
      SEQ_WAIT(20));    /* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  AD5940_EnterSleepS(); /* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  if (error == AD5940ERR_OK) {
    config.MeasureSeqInfo.SeqId = SEQID_0; // use SEQ0 to run this sequence
    config.MeasureSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
    config.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    config.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(config.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

// Function to handle interrupts
void sensor::EChem_CA::ISR(void) {
  if (!isRunning()) return; // Check that the technique is running

  std::vector<uint32_t> buf;

  // Read the FIFO
  Start_AD5940_SPI();
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
  AD5940_SleepKeyCtrlS(
      SLPKEY_LOCK); /* We need time to read data from FIFO, so, do not let AD5940 goes to hibernate automatically */
  if (AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE) {
    uint32_t numSamples = AD5940_FIFOGetCnt();
    buf.resize(numSamples);
    AD5940_FIFORd(buf.data(), numSamples);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Unlock so sequencer can put AD5940 to sleep */
    AD5940_EnterSleepS();
  }
  Stop_AD5940_SPI();

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
}

bool sensor::EChem_CA::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0xffff;
    push(calculateCurrent(pData[i], config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  }

  return true;
}

void sensor::EChem_CA::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}
