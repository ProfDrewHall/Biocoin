#include "sensors/EChem_SWV.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/Sensor.h"
#include "sensors/SensorManager.h"
#include "util/debug_log.h"

using namespace sensor;

// Structure for SWV parameters
struct SWV_PARAMETERS {
  float processingInterval; //[s] This controls how often the interrupt triggers and therefore, how often the MCU
                            // reads/processes the data.
  float maxCurrent;
  float Estart;      //[mv]
  float Estop;       //[mv]
  float Eamplitude;  //[mv]
  float Estep;       //[mv]
  float pulsePeriod; //[ms]
  uint8_t channel;
} __attribute__((packed));

EChem_SWV::EChem_SWV() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(SWV_PARAMETERS));

  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set

  config.SeqStartAddr = 0x10;     /* leave 16 commands for LFOSC calibration.  */
  config.MaxSeqLen = 1024 - 0x10; /* 4kB/4 = 1024  */
  config.RcalVal = 10000.0;       /* 10kOhm RCAL */
  config.ADCRefVolt = 1.82;       /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;

  config.PwrMod = AFEPWR_LP;
  config.AFEBW = AFEBW_50KHZ;     /*SWV does not need high BW*/
  config.ADCPgaGain = ADCPGA_1P5; /*Gain = 1.5V/V is the factory CVlibrated most accurate gain setting*/
  config.ADCSinc3Osr = ADCSINC3OSR_4;
  config.ADCSinc2Osr = ADCSINC2OSR_44; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.DataFifoSrc = DATATYPE_SINC2;

  config.LptiaRtiaSel = LPTIARTIA_10K; /* Maximum current decides RTIA value */
  config.LpTiaRf = LPTIARF_OPEN;
  config.LpTiaRl = LPTIARLOAD_SHORT;

  config.VzeroStart = 1300.0f; /* 1.3V */
  config.VzeroPeak = 1300.0f;  /* 1.3V */

  config.ExtRtiaVal = 10000000; // value of external TIA resistor (if used). Update as needed.
}

bool EChem_SWV::loadParameters(uint8_t* data, uint16_t len) {
  dbgInfo("Updating SWV parameters...");
  if (len != sizeof(SWV_PARAMETERS)) { // Check to ensure the size is correct
    dbgError(String("Incorrect payload size! Expected ") + String(sizeof(SWV_PARAMETERS)) + String(" but received ") +
             String(len));
    return false;
  }

  SWV_PARAMETERS params;
  memcpy(&params, data, sizeof(params));

  dbgInfo(String("\tProcessing Interval [s]: ") + String(params.processingInterval));
  dbgInfo(String("\tMax Current [mA]: ") + String(params.maxCurrent));
  dbgInfo(String("\tStarting Potential [mV]: ") + String(params.Estart));
  dbgInfo(String("\tStop Potential [mV]: ") + String(params.Estop));
  dbgInfo(String("\tPulse Amplitude [mV]: ") + String(params.Eamplitude));
  dbgInfo(String("\tStep Potential [mV]: ") + String(params.Estep));
  dbgInfo(String("\tPulse Period [ms]: ") + String(params.pulsePeriod));
  dbgInfo(String("\tChannel: ") + String(params.channel));

  // Bounds checking for parameters?

  // Set the parameters in the configuration structure
  config.Estart = params.Estart;
  config.Estop = params.Estop;
  config.Eamplitude = params.Eamplitude;
  config.Estep = params.Estep;
  config.PulsePeriod = params.pulsePeriod;
  config.PulseWidth = 0.5 * params.pulsePeriod; // Fixed 50% duty cycle for SWV

  config.FifoThresh = (uint32_t)(params.processingInterval * 1000 / config.PulseWidth);

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

bool EChem_SWV::start() {
  if (config.bParaChanged != bTRUE) return false; // Parameters have not been set

  clear();                       // Clear the data queue
  power::powerOnAFE(channel);    // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();            // Initialize SPI
  initAD5940();                  // Initialize the AD5940
  configureWaveformParameters(); // Define parameters for the measurement
  setupMeasurement();            // Initialize measurement sequence

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */

  /* Start it */
  WUPTCfg_Type wupt_cfg;
  wupt_cfg.WuptEn = bTRUE;
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.WuptOrder[1] = SEQID_2;
  wupt_cfg.WuptOrder[2] = SEQID_1;
  wupt_cfg.WuptOrder[3] = SEQID_2;

  // changing this adjusts the sample delay of the pulse
  wupt_cfg.SeqxSleepTime[SEQID_2] = 4;
  wupt_cfg.SeqxWakeupTime[SEQID_2] = (uint32_t)(LFOSCFreq * config.SampleDelay / 1000.0f) - 4 - 2;

  // changing this adjusts the "low time" of the pulse.
  wupt_cfg.SeqxSleepTime[SEQID_1] = 4;
  wupt_cfg.SeqxWakeupTime[SEQID_1] =
      (uint32_t)(LFOSCFreq * (config.PulsePeriod - config.PulseWidth - config.SampleDelay) / 1000.0f) - 4 - 2;

  // changing this adjusts the "high time" of the pulse.
  wupt_cfg.SeqxSleepTime[SEQID_0] = 4;
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(LFOSCFreq * (config.PulseWidth - config.SampleDelay) / 1000.0f) - 4 - 2;

  AD5940_WUPTCfg(&wupt_cfg);

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();
  return true;
}

bool EChem_SWV::stop() {
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
  rampState = SWVRampState::Stop;
  return true;
}

/* Initialize AD5940 basic blocks like clock */
int32_t EChem_SWV::initAD5940(void) {
  AD5940_HWReset();                                  // Hardware reset
  AD5940_Initialize();                               // Platform configuration
  AD5940_ConfigureClock();                           // Step 1 - Configure clock
  AD5940_ConfigureFIFO(FIFOSIZE_2KB, FIFOSRC_SINC3); // Step 2 - Configure FIFO and Sequencer
  AD5940_ConfigureSequencer(SEQMEMSIZE_4KB);
  AD5940_ConfigureInterrupts(AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0); // Step 3 - interrupts
  AD5940_ConfigureGPIO();              // Step 4 - Reconfigure GPIO
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); // Enable AFE to enter sleep mode
  AD5940_MeasureLFOSC(&LFOSCFreq);     // Measure the LFOSC frequency

  return 0;
}

/**
 * @brief Initialize the test. Call this function every time before starting  test.
 */
AD5940Err EChem_SWV::setupMeasurement(void) {
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
    config.RtiaCalValue.Phase = 0;
  } else {
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
  error = generateInitSequence(); /* initialization sequence */
  if (error != AD5940ERR_OK) return error;
  /* Generate sequence and write them to SRAM start from address config.SeqStartAddr */
  error = generateADCSequence(); /* ADC control sequence  */
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

AD5940Err EChem_SWV::configureLPLoop(void) {
  LPLoopCfg_Type lp_loop = {0};
  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = config.LpTiaRf;
  lp_loop.LpAmpCfg.LpTiaRload = config.LpTiaRl;
  if (config.ExtRtia == bTRUE) {
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(9) | LPTIASW(2) | LPTIASW(4) | LPTIASW(5) /*|LPTIASW(12)|LPTIASW(13)*/;
  } else {
    lp_loop.LpAmpCfg.LpTiaRtia = config.LptiaRtiaSel;
    // close SW5 to connect to external CTIA to ensure stability across different electrode double layer capacitances
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(2) | LPTIASW(4) /*|LPTIASW(12)|LPTIASW(13)*/;
  }

  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  // just initial DAC settings before SWV truly begins
  lp_loop.LpDacCfg.DacData6Bit =
      (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB); // WE voltage
  lp_loop.LpDacCfg.DacData12Bit =
      (int32_t)(lp_loop.LpDacCfg.DacData6Bit * 64 +
                (config.RampStartVolt - config.Eamplitude + config.Estep) / AD5940_12BIT_DAC_1LSB); // RE voltage

  if (lp_loop.LpDacCfg.DacData12Bit < lp_loop.LpDacCfg.DacData6Bit * 64) lp_loop.LpDacCfg.DacData12Bit--;
  // truncate if needed
  if (lp_loop.LpDacCfg.DacData12Bit > 4095) lp_loop.LpDacCfg.DacData12Bit = 4095;
  if (lp_loop.LpDacCfg.DacData6Bit > 63) lp_loop.LpDacCfg.DacData6Bit = 63;

  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA /*|LPDACSW_VBIAS2PIN*/ | LPDACSW_VZERO2LPTIA /*|LPDACSW_VZERO2PIN*/;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lp_loop);

  return AD5940ERR_OK;
}

void EChem_SWV::configureWaveformParameters() {
  config.bSqrWaveHiLevel = bTRUE;
  rampState = SWVRampState::Start; // Reset the ramp state
  config.bFirstDACSeq = bTRUE;

  config.SampleDelay = 0.75 * config.PulseWidth;

  //   Vbias = -Ewe, so RE ramps opposite the desired WE–RE sweep
  const float dir = (config.Estop > config.Estart) ? 1.0f : -1.0f;
  config.RampStartVolt = -config.Estart + dir * (-2.0f * config.Eamplitude) + dir * config.Estep;
  config.RampPeakVolt = -config.Estop + dir * (-2.0f * config.Eamplitude);
}

/* Generate init sequence for CV. This runs only one time. */
AD5940Err EChem_SWV::generateInitSequence(void) {
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
AD5940Err EChem_SWV::generateADCSequence(void) {
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
  AD5940_SEQGenInsert(SEQ_WAIT(20));                                             /* allow data to reach FIFO */
  AD5940_EnterSleepS();                                                          /* Hibernate */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if (error != AD5940ERR_OK) return error;

  AD5940_StructInit(&config.ADCSeqInfo, sizeof(config.ADCSeqInfo));
  if (SeqLen >= config.MaxSeqLen) return AD5940ERR_SEQLEN; /* obviously too large */

  config.ADCSeqInfo.SeqId = SEQID_2;
  config.ADCSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
  config.ADCSeqInfo.pSeqCmd = pSeqCmd;
  config.ADCSeqInfo.SeqLen = SeqLen;
  config.ADCSeqInfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&config.ADCSeqInfo);

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
AD5940Err EChem_SWV::generateDACSequence(void) {
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

  const float dir = (config.Estop > config.Estart) ? -1.0f : +1.0f; // because Vbias = -Ewe
  const float stepAbs = fabsf(config.Estep);
  const float pulseAbs = fabsf(config.Eamplitude);
  config.StepNumber = (uint32_t)(2.0f * round((fabsf(config.RampPeakVolt - config.RampStartVolt) / stepAbs)));

  /* Do some math calculations */
  if (config.bFirstDACSeq == bTRUE) {
    /* Reset bIsFirstRun at end of function. */
    int32_t DACSeqLenMax;
    StepsRemainning = config.StepNumber;
    DACSeqLenMax = (int32_t)config.MaxSeqLen - (int32_t)config.ADCSeqInfo.SeqLen - 
                   (int32_t)config.ADCSeqInfo.SeqLen;
    if (DACSeqLenMax < SEQLEN_ONESTEP * 4) return AD5940ERR_SEQLEN; /* No enough sequencer SRAM available */
    DACSeqLenMax -= SEQLEN_ONESTEP * 2;                             /* Reserve commands each block */
    StepsPerBlock = DACSeqLenMax / SEQLEN_ONESTEP / 2;
    DACSeqBlk0Addr = config.ADCSeqInfo.SeqRamAddr + config.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock * SEQLEN_ONESTEP;
    DACSeqCurrBlk = CURRBLK_BLK0;

    /* Analog part */
    config.DACCodePerStep = dir * (pulseAbs / AD5940_12BIT_DAC_1LSB);
    config.DACCodePerRamp = dir * (stepAbs / AD5940_12BIT_DAC_1LSB);

#if ALIGIN_VOLT2LSB
    config.DACCodePerStep = (int32_t)config.DACCodePerStep;
    config.DACCodePerRamp = (int32_t)config.DACCodePerRamp;
#endif

    config.CurrRampCode =
        config.RampStartVolt / AD5940_12BIT_DAC_1LSB + (config.DACCodePerStep - config.DACCodePerRamp);

    rampState = SWVRampState::Start; // Reset the ramp state
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
void EChem_SWV::ISR(void) {
  if (!isRunning() && rampState != SWVRampState::Stop) return;

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

    // generateDACSequence();
    stop();
    updateStatus(TestState::NOT_RUNNING); // Update the test state
  }

  Stop_AD5940_SPI();

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
}

// Calculate DAC code step by step
AD5940Err EChem_SWV::updateRampDACCode(uint32_t* pDACData) {
  if (pDACData == nullptr) return AD5940ERR_PARA;

  // 1) Handle state transitions (only when thresholds are crossed)
  switch (rampState) {
  case SWVRampState::Start:
    config.CurrVzeroCode = (uint32_t)((config.VzeroStart - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    rampState = SWVRampState::State1;
    break;
  case SWVRampState::State1:
    if (config.CurrStepPos >= config.StepNumber / 2) {
      config.CurrVzeroCode = (uint32_t)((config.VzeroPeak - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
      rampState = SWVRampState::State2;
    }
    break;
  case SWVRampState::State2:
    if (config.CurrStepPos >= config.StepNumber) rampState = SWVRampState::Stop;
    break;
  case SWVRampState::Stop:
    break;
  }

  // 2) Advance step position and ramp code
  if (config.bSqrWaveHiLevel) {
    config.CurrRampCode -= (config.DACCodePerStep - config.DACCodePerRamp);
    config.bSqrWaveHiLevel = bFALSE;
  } else {
    if (config.CurrStepPos == 1) { // first DAC rising edge uses half-step
      config.CurrRampCode += config.DACCodePerStep / 2;
    } else {
      config.CurrRampCode += config.DACCodePerStep;
    }
    config.bSqrWaveHiLevel = bTRUE;
  }

  config.CurrStepPos++;

  // 3) Compose VZERO (6b) + VBIAS (12b), with saturation
  uint32_t VzeroCode = config.CurrVzeroCode;
  uint32_t VbiasCode = (uint32_t)(VzeroCode * 64 + config.CurrRampCode);
  if (VbiasCode < (VzeroCode * 64)) VbiasCode--;

  /* Truncate */
  if (VbiasCode > 4095) VbiasCode = 4095;
  if (VzeroCode > 63) VzeroCode = 63;

  *pDACData = (VzeroCode << 12) | VbiasCode;
  return AD5940ERR_OK;
}

bool EChem_SWV::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0xffff;
    push(calculateCurrent(pData[i], config.ADCPgaGain, config.ADCRefVolt, config.RtiaCalValue.Magnitude));
  }

  return true;
}

void EChem_SWV::printResult(void) {
  forEach([](const float& i_uA) { Serial.printf("    I = %.5f uA\n", i_uA); });
}
