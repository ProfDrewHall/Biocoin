#include "sensors/EChem_CV.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"
#include "util/debug_log.h"


// Structure for how parameters are passed down from the host
struct CV_PARAMETERS {
  float processingInterval; //[s] This controls how often the interrupt triggers and therefore, how often the MCU
                            // reads/processes the data.
  float maxCurrent;
  float Estart;     //[mv]
  float Evertex1;   //[mv]
  float Evertex2;   //[mv]
  float Estep;      //[mv]
  float pulseWidth; //[ms]
  uint8_t channel;  // Channel (0-3) to select
} __attribute__((packed));

sensor::EChem_CV::EChem_CV() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(CVConfig_Type));

  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set

  /* Step1: configure general parameters */
  config.SeqStartAddr = 0x10;     /* leave 16 commands for LFOSC calibration.  */
  config.MaxSeqLen = 1024 - 0x10; /* 4kB/4 = 1024  */
  config.RcalVal = 10000.0;       /* 10kOhm RCAL */
  config.ADCRefVolt = 1.82;       /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;     /*CV does not need high BW*/
  config.ADCPgaGain = ADCPGA_1P5; /*Gain = 1.5V/V is the factory CVlibrated most accurate gain setting*/
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
  config.SampleDelay = 1.0f; /* 1ms */
  config.VzeroStart = 1300.0f;
  config.VzeroPeak = 1300.0f;

  config.ExtRtiaVal = 10000000; // value of external TIA resistor (if used). Update as needed.

  config.bTestFinished = bFALSE;

  config.bFirstDACSeq = bTRUE;

  rampState = RampState::Start; // Initialize the ramp state
  channel = 0;
}

bool sensor::EChem_CV::loadParameters(uint8_t* data, uint16_t len) {
  dbgInfo("Updating CV parameters...");
  if (len != sizeof(CV_PARAMETERS)) { // Check to ensure the size is correct
    dbgError(String("Incorrect payload size! Expected ") + String(sizeof(CV_PARAMETERS)) + String(" but received ") +
              String(len));
    return false;
  }

  CV_PARAMETERS params;
  memcpy(&params, data, sizeof(params));

  dbgInfo(String("\tProcessing Interval [s]: ") + String(params.processingInterval));
  dbgInfo(String("\tMax Current [mA]: ") + String(params.maxCurrent));
  dbgInfo(String("\tStarting Potential [mV]: ") + String(params.Estart));
  dbgInfo(String("\tVertex1 Potential [mV]: ") + String(params.Evertex1));
  dbgInfo(String("\tVertex2 Potential [mV]: ") + String(params.Evertex2));
  dbgInfo(String("\tStep Potential [mV]: ") + String(params.Estep));
  dbgInfo(String("\tPulse Width [ms]: ") + String(params.pulseWidth));
  dbgInfo(String("\tChannel: ") + String(params.channel));

  // Bounds checking for parameters?

  // Set the parameters in the configuration structure
  config.Estart = params.Estart;
  config.Evertex1 = params.Evertex1;
  config.Evertex2 = params.Evertex2;
  config.Estep = params.Estep;
  config.PulseWidth = params.pulseWidth;
  config.FifoThresh = (uint32_t)(params.processingInterval * 1000 / params.pulseWidth);

  // Current range

  // Bounds/validity checking of parameters
  /*
    if (params.processing_interval < params.sampling_interval) {
      dbgError("Processing interval needs to be more than sampling interval.");
      return false;
    }

    // Still need to use max_current to calculate the gain resistor
    // config.LptiaRtiaSel = LPTIARTIA_10K;		// this sets the current range for the experiment
    // End Fix

  */

  channel = params.channel;

  config.bParaChanged = bTRUE;

  return true;
}

bool sensor::EChem_CV::start() {
  if (config.bParaChanged != bTRUE) return false; // Parameters have not been set

  clear();                    // Clear the data queue
  power::powerOnAFE(channel); // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();         // Initialize SPI
  initAD5940();               // Initialize the AD5940
  configureRampParameters();  // Define parameters for the measurement
  setupMeasurement();         // Initialize measurement sequence

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */
  /**
   *  RAMP example is special, because the sequence is dynamically generated.
   *  Before 'START' ramp test, call AppRAMPInit firstly.
   */
  if (rampState == RampState::Stop) return false;

  /* Start it */
  WUPTCfg_Type wupt_cfg;
  wupt_cfg.WuptEn = bTRUE;
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.WuptOrder[1] = SEQID_2;
  wupt_cfg.WuptOrder[2] = SEQID_1;
  wupt_cfg.WuptOrder[3] = SEQID_2;
  wupt_cfg.SeqxSleepTime[SEQID_2] = 4;
  wupt_cfg.SeqxWakeupTime[SEQID_2] = (uint32_t)(LFOSCFreq * config.SampleDelay / 1000.0f) - 4 - 2;
  wupt_cfg.SeqxSleepTime[SEQID_0] = 4;
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(LFOSCFreq * (config.PulseWidth - config.SampleDelay) / 1000.0f) - 4 - 2;
  wupt_cfg.SeqxSleepTime[SEQID_1] = wupt_cfg.SeqxSleepTime[SEQID_0];
  wupt_cfg.SeqxWakeupTime[SEQID_1] = wupt_cfg.SeqxWakeupTime[SEQID_0];
  AD5940_WUPTCfg(&wupt_cfg);

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();

  return true;
}

bool sensor::EChem_CV::stop() {
  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */
  /* Start Wupt right now */
  AD5940_WUPTCtrl(bFALSE);
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
int32_t sensor::EChem_CV::initAD5940(void) {
  AD5940_HWReset();                                  // Hardware reset
  AD5940_Initialize();                               // Platform configuration
  AD5940_ConfigureClock();                           // Step 1 - Configure clock
  AD5940_ConfigureFIFO(FIFOSIZE_2KB, FIFOSRC_SINC3); // Step 2 - Configure FIFO and Sequencer
  AD5940_ConfigureSequencer(SEQMEMSIZE_2KB);
  AD5940_ConfigureInterrupts(AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0); // Step 3 - interrupts
  AD5940_ConfigureGPIO();              // Step 4 - Reconfigure GPIO
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); // Enable AFE to enter sleep mode
  AD5940_MeasureLFOSC(&LFOSCFreq);     // Measure the LFOSC frequency

  return 0;
}

/**
 * @brief Initialize the test. Call this function every time before starting  test.
 */
AD5940Err sensor::EChem_CV::setupMeasurement(void) {
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB; /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  if (seq_buffer == 0) return AD5940ERR_PARA;
  if (SEQ_BUFF_SIZE == 0) return AD5940ERR_PARA;

  if (config.LptiaRtiaSel == LPTIARTIA_OPEN) { /* Internal RTIA is opened. User wants to use external RTIA resistor */
    config.RtiaCalValue.Magnitude = config.ExtRtiaVal;
  }
  /* Do RTIA calibration */
  if (config.ExtRtia == bFALSE) {
    AD5940_CalibrateRTIA(config.AdcClkFreq, config.SysClkFreq, config.LptiaRtiaSel, config.RcalVal,
                         &config.RtiaCalValue);
  }

  /* Reconfigure FIFO, The Rtia calibration function may generate data that stored to FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = config.FifoThresh; /* Change FIFO paramters */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  AD5940_FIFOCfg(&fifo_cfg);

  /* Clear all interrupts */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  AD5940_SEQGenInit(seq_buffer, SEQ_BUFF_SIZE);
  /* Generate sequence and write them to SRAM start from address AppCVCfg.SeqStartAddr */
  error = generateInitSequence(); /* initialization sequence */
  if (error != AD5940ERR_OK) return error;
  error = generateADCSequence(); /* ADC control sequence */
  if (error != AD5940ERR_OK) return error;

  /* Generate DAC sequence */
  config.bFirstDACSeq = bTRUE;
  error = generateDACSequence();
  if (error != AD5940ERR_OK) return error;

  /* Configure sequence info. */
  config.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.InitSeqInfo);

  AD5940_SEQCtrlS(bTRUE); /* Enable sequencer */
  AD5940_SEQMmrTrig(config.InitSeqInfo.SeqId);
  while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE)
    ;
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

  config.ADCSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.ADCSeqInfo);

  config.DACSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&config.DACSeqInfo);

  AD5940_SEQCtrlS(bFALSE);
  AD5940_WriteReg(REG_AFE_SEQCNT, 0);
  AD5940_SEQCtrlS(bTRUE); /* Enable sequencer, and wait for trigger */

  AD5940_AFEPwrBW(config.PwrMod, config.AFEBW); /* Set to low power mode */

  return AD5940ERR_OK;
}

void sensor::EChem_CV::configureRampParameters(void) {
  config.SampleDelay = 0.75 * config.PulseWidth; /*Time between update DAC and ADC sample. Unit is ms. Sampling occurs
                                                    at the last 25% of a pulse width by default*/
  config.RampStartVolt = config.Estart;
  config.RampPeakVolt = config.Evertex1;

  // user wants to do forward direction ramp to begin
  if (config.RampPeakVolt > config.RampStartVolt) {
    config.Estart = config.Estart - config.Estep; // go one step further to ensure startpoint
    config.RampStartVolt = -1 * config.Estart;
    config.RampPeakVolt = -1 * config.Evertex1;

    // Segment-specific step counts
    // Depending on Evertex2 relative to Estart value, deal with endpoints appropriately
    if (config.Evertex2 == (config.Estart + config.Estep)) {
      float dV1 = fabsf(config.Evertex1 - config.Estart); // Estart to Evertex1
      float dV2 =
          fabsf(config.Evertex2 - config.Evertex1 +
                config.Estep); // Evertex1 to Evertex2. End one point before Estart, rather than one point after.
      config.StepNumberSeg1 = (uint32_t)(round(dV1 / config.Estep));
      config.StepNumberSeg2 = (uint32_t)(round(dV2 / config.Estep));
      config.StepNumberSeg3 = 0;
    } else {
      float dV1 = fabsf(config.Evertex1 - config.Estart);   // Estart to Evertex1
      float dV2 = fabsf(config.Evertex2 - config.Evertex1); // Evertex1 to Evertex2. End one point after Estart.
      float dV3 = fabsf(config.Estart - config.Evertex2);   // Evertex2 to Estart
      config.StepNumberSeg1 = (uint32_t)(round(dV1 / config.Estep));
      config.StepNumberSeg2 = (uint32_t)(round(dV2 / config.Estep));
      config.StepNumberSeg3 = (uint32_t)(round(dV3 / config.Estep));
    }
    config.StepNumber = config.StepNumberSeg1 + config.StepNumberSeg2 + config.StepNumberSeg3; /* Total steps.*/
  }
  // user wants to do reverse direction ramp to begin
  else {
    config.Estart = config.Estart + config.Estep; // go one step further to ensure startpoint
    config.RampStartVolt = -1 * config.Estart;
    config.RampPeakVolt = -1 * config.Evertex1;

    // Segment-specific step counts
    // Depending on Evertex2 relative to Estart value, deal with endpoints appropriately
    if (config.Evertex2 == (config.Estart - config.Estep)) {
      float dV1 = fabsf(config.Evertex1 - config.Estart); // Estart to Evertex1
      float dV2 =
          fabsf(config.Evertex2 - config.Evertex1 -
                config.Estep); // Evertex1 to Evertex2. End one point before Estart, rather than one point after.
      config.StepNumberSeg1 = (uint32_t)(round(dV1 / config.Estep));
      config.StepNumberSeg2 = (uint32_t)(round(dV2 / config.Estep));
      config.StepNumberSeg3 = 0;
    } else {
      float dV1 = fabsf(config.Evertex1 - config.Estart);   // Estart to Evertex1
      float dV2 = fabsf(config.Evertex2 - config.Evertex1); // Evertex1 to Evertex2. End one point after Estart.
      float dV3 = fabsf(config.Estart - config.Evertex2);   // Evertex2 to Estart
      config.StepNumberSeg1 = (uint32_t)(round(dV1 / config.Estep));
      config.StepNumberSeg2 = (uint32_t)(round(dV2 / config.Estep));
      config.StepNumberSeg3 = (uint32_t)(round(dV3 / config.Estep));
    }
    config.StepNumber = config.StepNumberSeg1 + config.StepNumberSeg2 + config.StepNumberSeg3; /* Total steps.*/
    config.Estep = -1 * config.Estep;
  }
}

/* Generate init sequence for CV. This runs only one time. */
AD5940Err sensor::EChem_CV::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator here
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all to disable state

  AD5940_ConfigureAFEReferences(true, true, true, true);
  AD5940_ConfigureLPLoop(config.VzeroStart, config.Estart + config.Estep, config.LpTiaRf, config.LpTiaRl,
                         config.ExtRtia, config.LptiaRtiaSel);
  AD5940_ConfigureDSP(ADCMUXN_LPTIA0_N, ADCMUXP_LPTIA0_P, config.ADCPgaGain, config.ADCSinc2Osr, config.ADCSinc3Osr);

  HSLoopCfg_Type hs_loop = {0};
  AD5940_HSLoopCfgS(&hs_loop);

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0);

  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we
                                      only want it to run one time. */

  /* Stop sequence generator here */
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  if (error == AD5940ERR_OK) {
    AD5940_StructInit(&config.InitSeqInfo, sizeof(config.InitSeqInfo));
    if (SeqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN;

    config.InitSeqInfo.SeqId = SEQID_3;
    config.InitSeqInfo.SeqRamAddr = config.SeqStartAddr;
    config.InitSeqInfo.pSeqCmd = pSeqCmd;
    config.InitSeqInfo.SeqLen = SeqLen;
    config.InitSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&config.InitSeqInfo);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

/**
 * @brief Generate ADC control sequence and write the commands to SRAM.
 * @return return error code.
 */
AD5940Err sensor::EChem_CV::generateADCSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  const uint32_t* pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataCount = 1; /* Sample one point everytime */
  clks_cal.DataType = config.DataFifoSrc;
  clks_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  clks_cal.ADCAvgNum = 0; /* Don't care */
  clks_cal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));                                       /* wait 250us for reference power up */
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bTRUE);                   /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));                                       /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_SINC2NOTCH, bFALSE); /* Stop ADC */
  AD5940_SEQGenInsert(
      SEQ_WAIT(20));    /* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  AD5940_EnterSleepS(); /* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  if (error == AD5940ERR_OK) {
    AD5940_StructInit(&config.ADCSeqInfo, sizeof(config.ADCSeqInfo));
    if ((SeqLen + config.InitSeqInfo.SeqLen) >= config.MaxSeqLen) return AD5940ERR_SEQLEN;
    config.ADCSeqInfo.SeqId = SEQID_2;
    config.ADCSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
    config.ADCSeqInfo.pSeqCmd = pSeqCmd;
    config.ADCSeqInfo.SeqLen = SeqLen;
    config.ADCSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&config.ADCSeqInfo);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

/**
 * @brief Update DAC sequence in SRAM in real time.
 * @details This function generates sequences to update DAC code step by step. It's also called in interrupt
 *          function when half commands in SRAM has been completed. We don't use sequence generator to save memory.
 *          Check more details from documentation of this example. @ref Ramp_Test_Example
 * @return return error code
 *
 * */
AD5940Err sensor::EChem_CV::generateDACSequence(void) {
#define SEQLEN_ONESTEP 4L /* How many sequence commands are needed to update LPDAC. */
#define CURRBLK_BLK0 0 /* Current block is BLOCK0 */
#define CURRBLK_BLK1 1 /* Current block is BLOCK1 */

  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t DACData, SRAMAddr;
  uint32_t i;
  uint32_t StepsThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];

  /* All below static variables are inited in below 'if' block. They are only used in this function */
  static BoolFlag bCmdForSeq0 = bTRUE;
  static uint32_t DACSeqBlk0Addr, DACSeqBlk1Addr;
  static uint32_t StepsRemainning, StepsPerBlock, DACSeqCurrBlk;

  /* Do some math calculations */
  if (config.bFirstDACSeq == bTRUE) {
    /* Reset bIsFirstRun at end of function. */
    int32_t DACSeqLenMax;
    StepsRemainning = config.StepNumber;
    DACSeqLenMax = (int32_t)config.MaxSeqLen - (int32_t)config.InitSeqInfo.SeqLen - (int32_t)config.ADCSeqInfo.SeqLen;
    if (DACSeqLenMax < SEQLEN_ONESTEP * 4) return AD5940ERR_SEQLEN; /* No enough sequencer SRAM available */
    DACSeqLenMax -= SEQLEN_ONESTEP * 2;                             /* Reserve commands each block */
    StepsPerBlock = DACSeqLenMax / SEQLEN_ONESTEP / 2;
    DACSeqBlk0Addr = config.ADCSeqInfo.SeqRamAddr + config.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock * SEQLEN_ONESTEP;
    DACSeqCurrBlk = CURRBLK_BLK0;

    /* Analog part */
    config.DACCodePerStep = config.Estep / AD5940_12BIT_DAC_1LSB;

#if ALIGIN_VOLT2LSB
    config.DACCodePerStep = (int32_t)config.DACCodePerStep;
#endif
    if (config.DACCodePerStep > 0)
      config.bDACCodeInc = bTRUE;
    else {
      config.DACCodePerStep = -config.DACCodePerStep; /* Always positive */
      config.bDACCodeInc = bFALSE;
    }

    config.CurrRampCode = config.RampStartVolt / AD5940_12BIT_DAC_1LSB;
    rampState = RampState::Start; // Reset the ramp state
    config.CurrStepPos = 0;
    bCmdForSeq0 = bTRUE; /* Start with SEQ0 */
  }

  if (StepsRemainning == 0) return AD5940ERR_OK; /* Done. */
  bIsFinalBlk = StepsRemainning <= StepsPerBlock ? bTRUE : bFALSE;
  StepsThisBlock = bIsFinalBlk ? StepsRemainning : StepsPerBlock;
  StepsRemainning -= StepsThisBlock;

  BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
  SRAMAddr = BlockStartSRAMAddr;

  for (i = 0; i < StepsThisBlock - 1; i++) {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr += SEQLEN_ONESTEP; /* Jump to next sequence */
    updateRampDACCode(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }
  /* Add final DAC update */
  if (bIsFinalBlk) {
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr += SEQLEN_ONESTEP;
    /* After update LPDAC with final data, we let sequencer to run 'final final' command, to disable sequencer.  */
    updateRampDACCode(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(
        10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    CurrAddr += SEQLEN_ONESTEP;
    /* The final final command is to disable sequencer. */
    SeqCmdBuff[0] = SEQ_NOP(); // do nothing
    SeqCmdBuff[1] = SEQ_NOP();
    SeqCmdBuff[2] = SEQ_NOP();
    SeqCmdBuff[3] = SEQ_STOP(); // stop sequencer
    /* Disable sequencer, END of sequencer interrupt is generated. */
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
  } else /* This is not the final block */
  {
    /* Jump to next block. */
    uint32_t CurrAddr = SRAMAddr;
    SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;
    updateRampDACCode(&DACData);
    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(
        10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_INT0(); /* Generate Custom interrupt 0. */
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0; /* Switch between Block0 and block1 */

  if (config.bFirstDACSeq) {
    config.bFirstDACSeq = bFALSE;
    if (bIsFinalBlk == bFALSE) {
      /* Otherwise there is no need to init block1 sequence */
      error = generateDACSequence();
      if (error != AD5940ERR_OK) return error;
    }
    /* This is the first DAC sequence. */
    config.DACSeqInfo.SeqId = SEQID_0;
    config.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
    config.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
    config.DACSeqInfo.WriteSRAM = bFALSE; /* No need to write to SRAM. We already write them above. */
    AD5940_SEQInfoCfg(&config.DACSeqInfo);
  }

  return AD5940ERR_OK;
}

// Function to handle interrupts
void sensor::EChem_CV::ISR(void) {
  if (!isRunning() && rampState != RampState::Stop) return; // Check that the technique is running

  std::vector<uint32_t> buf;
  uint32_t numSamples = 0;

  // Read the FIFO
  Start_AD5940_SPI();
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
  AD5940_SleepKeyCtrlS(
      SLPKEY_LOCK); /* We need time to read data from FIFO, so, do not let AD5940 goes to hibernate automatically */

  uint32_t interruptFlag = AD5940_INTCGetFlag(AFEINTC_0);
  if (interruptFlag & AFEINTSRC_CUSTOMINT0) /* High priority. */
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
    generateDACSequence();
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    AD5940_EnterSleepS(); /* If there is need to do AFE re-configure, do it here when AFE is in active state */
  }
  if (interruptFlag & AFEINTSRC_DATAFIFOTHRESH) {
    numSamples = AD5940_FIFOGetCnt();
    buf.resize(numSamples);
    AD5940_FIFORd(buf.data(), numSamples);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Unlock so sequencer can put AD5940 to sleep */
    AD5940_EnterSleepS();
  }

  if (interruptFlag & AFEINTSRC_ENDSEQ) {
    numSamples = AD5940_FIFOGetCnt();
    buf.resize(numSamples);
    AD5940_FIFORd(buf.data(), numSamples);
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

    /* Reset variables so measurement can be restarted*/
    rampState = RampState::Start; // Reset the ramp state
    config.bFirstDACSeq = bTRUE;
    config.bDACCodeInc = bTRUE;
    // generateDACSequence();

    stop();
    updateStatus(TestState::NOT_RUNNING); // Update the test state
  }

  Stop_AD5940_SPI();

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
}

// Calculate DAC code step by step.
AD5940Err sensor::EChem_CV::updateRampDACCode(uint32_t* pDACData) {
  if (pDACData == nullptr) return AD5940ERR_PARA;

  // 1) Handle state transitions (only when thresholds are crossed)
  switch (rampState) {
  case RampState::Start: // Estart -> Evertex1
    config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    config.bDACCodeInc = (config.RampPeakVolt > config.RampStartVolt) ? bTRUE : bFALSE;
    rampState = RampState::Vertex1;
    break;

  case RampState::Vertex1: // Evertex1 -> Evertex2
    if (config.CurrStepPos >= config.StepNumberSeg1) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroPeak - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      config.bDACCodeInc = (config.RampPeakVolt > -config.Evertex2) ? bFALSE : bTRUE;
      rampState = RampState::Vertex2;
    }
    break;

  case RampState::Vertex2: // Evertex2 -> Estart
    if (config.CurrStepPos >= (config.StepNumberSeg1 + config.StepNumberSeg2)) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      config.bDACCodeInc = (config.RampStartVolt > -config.Evertex2) ? bTRUE : bFALSE;
      rampState = RampState::Return;
    }
    break;

  case RampState::Return:
    if (config.CurrStepPos >= config.StepNumber) rampState = RampState::Stop;
    break;

  case RampState::Stop:
    break;
  }

  // 2) Advance step position and ramp code
  config.CurrStepPos++;
  config.CurrRampCode += (config.bDACCodeInc ? +config.DACCodePerStep : -config.DACCodePerStep);

  // 3) Compose VZERO (6b) + VBIAS (12b), with saturation
  uint32_t VzeroCode = std::min<uint32_t>(config.CurrVzeroCode, 63); // VzeroCode is 6 bits, max value is 63
  uint32_t VbiasCode = (uint32_t)(VzeroCode * 64 + config.CurrRampCode);
  if (VbiasCode < (VzeroCode * 64)) VbiasCode--;
  if (VbiasCode > 4095) VbiasCode = 4095;

  *pDACData = (VzeroCode << 12) | VbiasCode;
  return AD5940ERR_OK;
}

bool sensor::EChem_CV::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0xffff;
    push(calculateCurrent(pData[i], config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  }

  return true;
}

void sensor::EChem_CV::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}
