#include "sensors/EChem_Imp.h"

#include "HWConfig/constants.h"
#include "drivers/ad5940_hal.h"
#include "power/power.h"
#include "sensors/Sensor.h"
#include "util/debug_log.h"

using namespace sensor;

// Structure for how parameters are passed down from the host
struct IMP_PARAMETERS {
  float samplingInterval;   //[s] how often the ADC samples by controlling the sleep time between sequences
  float processingInterval; //[s] how often the interrupt triggers and thus, how often the MCU reads/processes the data
  uint8_t IMP_4wire;        // flag indicating if running 4-wire (true) or 2-wire measurement (false)
  uint8_t AC_coupled;       // flag indicating if measurement is AC coupled (true) or DC coupled (false)
  float maxCurrent;
  float Eac;       //[mv]
  float frequency; //[Hz]
} __attribute__((packed));

EChem_Imp::EChem_Imp() {
  // Initialize structures to known values
  memset(&config, 0, sizeof(ImpConfig_Type));
  config.bParaChanged = bFALSE; // Flag used to indicate parameters have been set
  config.SeqStartAddr = 0;

  config.SysClkFreq = SYS_CLOCK_FREQ / 4;
  config.AdcClkFreq = 16000000.0;
  config.SamplingInterval = 30.0f; // Default value 30s between samples

  config.RcalVal = 10000.0; // 10kOhm on Biocoin
  // Switch/pin config settings
  config.DswitchSel = SWD_CE0;  // positive force pin for Impedance measurement
  config.PswitchSel = SWP_CE0;  // positive force pin for Impedance measurement
  config.NswitchSel = SWN_AIN1; // negative force pin for Impedance measurement
  config.TswitchSel = SWT_AIN1; // negative force pin for Impedance measurement
  config.SenseP = ADCMUXP_AIN3; // positive sense pin for Impedance measurement, don't care if using 2-wire
  config.SenseN = ADCMUXN_AIN2; // negative sense pin for Impedance measurement, don't care if using 2-wire

  config.PwrMod = AFEPWR_HP;
  config.AFEBW = AFEBW_250KHZ;
  config.ADCPgaGain = ADCPGA_1P5; /*Gain = 1.5V/V is the factory calibrated most accurate gain setting*/
  config.ADCSinc3Osr = ADCSINC3OSR_2;
  config.ADCSinc2Osr = ADCSINC2OSR_22; // adjust these as needed if really fast or really slow sampling is required.
                                       // Power vs. SNR tradeoff.
  config.HstiaRtiaSel = HSTIARTIA_5K;

  config.CtiaSel = 16;
  config.ExcitBufGain = EXCITBUFGAIN_2, config.HsDacGain = HSDACGAIN_1, config.HsDacUpdateRate = 7;
  config.DacVoltPP = 800.0;

  config.DftNum = DFTNUM_8192;
  config.DftSrc = DFTSRC_SINC3;
  config.HanWinEn = bTRUE;

  config.SweepCfg.SweepEn = bFALSE;
  config.SweepCfg.SweepStart = 1000;
  config.SweepCfg.SweepStop = 100000.0;
  config.SweepCfg.SweepPoints = 101;
  config.SweepCfg.SweepLog = bTRUE;
  config.SweepCfg.SweepIndex = 0;

  config.StopRequired = bFALSE;

  config.MeasSeqCycleCount = 0;
}

bool EChem_Imp::loadParameters(uint8_t* data, uint16_t len) {
  dbgInfo("Updating IMP parameters...");
  if (len != sizeof(IMP_PARAMETERS)) { // Check to ensure the size is correct
    dbgError(String("Incorrect payload size! Expected ") + String(sizeof(IMP_PARAMETERS)) + String(" but received ") +
             String(len));
    return false;
  }

  IMP_PARAMETERS params;
  memcpy(&params, data, sizeof(params));

  dbgInfo(String("\tSampling Interval [s]: ") + String(params.samplingInterval));
  dbgInfo(String("\tProcessing Interval [s]: ") + String(params.processingInterval));
  dbgInfo(String("\tMax Current [mA]: ") + String(params.maxCurrent));
  dbgInfo(String("\tEac Potential [mV]: ") + String(params.Eac));
  dbgInfo(String("\tFrequency [Hz]: ") + String(params.frequency));
  dbgInfo(String("\t4-wire Measurement: ") + String(params.IMP_4wire ? "Enabled" : "Disabled"));
  dbgInfo(String("\tAC-coupled Measurement: ") + String(params.AC_coupled ? "True" : "False"));

  // Bounds/validity checking of parameters
  if (params.processingInterval < params.samplingInterval) {
    dbgError("Processing interval needs to be more than sampling interval.");
    return false;
  }

  // The threshold to set when the interrupt triggers. Number of samples required to reach desired processing time is
  // processing time divided by sampling interval
  config.FifoThresh = (uint32_t)(4u * params.processingInterval / params.samplingInterval);
  if (config.FifoThresh < 4u) config.FifoThresh = 4u;
  config.FifoThresh &= ~0x3u; // Ensure it is a multiple of 4

  config.SamplingInterval = params.samplingInterval;

  config.IMP4WIRE = static_cast<BoolFlag>(params.IMP_4wire);   // Specify if 4-wire or 2-wire Impedance measurement
  config.ACcoupled = static_cast<BoolFlag>(params.AC_coupled); // Specify if operating in an AC-coupled scenario
  config.Eac = params.Eac;                                     // mV amplitude (peak)
  config.SinFreq = params.frequency;                           // Hz

  // Still need to use max_current to calculate the gain resistor
  // config.LptiaRtiaSel = LPTIARTIA_10K;		// this sets the current range for the experiment

  config.bParaChanged = bTRUE;

  return true;
}

bool EChem_Imp::start() {
  if (config.bParaChanged != bTRUE) return false; // Parameters have not been set

  clear();                       // Clear the data queue
  power::powerOnAFE(0);          // Turn on the power to the AD5940, select the correct mux input
  Start_AD5940_SPI();            // Initialize SPI
  initAD5940();                  // Initialize the AD5940
  configureWaveformParameters(); // Define parameters for the measurement
  setupMeasurement();            // Initialize measurement sequence

  if (AD5940_WakeUp(10) > 10) /* Wakeup AFE by read register, read 10 times at most */
    return false;             /* Wakeup Failed */

  /* Configure Wakeup Timer*/
  // configure to trigger above sequence periodically to measure data.
  WUPTCfg_Type wupt_cfg;
  wupt_cfg.WuptEn = bTRUE;
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.SeqxSleepTime[SEQID_0] = 1; //  minimum value is 1 (2x 32kHz clock). Do not set it to zero.
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(LFOSCFreq * config.SamplingInterval) - 2 - 1;
  AD5940_WUPTCfg(&wupt_cfg); // will enable Wakeup timer, measurement begins here
  AD5940_EnterSleepS(); // Enter Hibernate now otherwise it won't start sleeping until after the first interrupt period
  config.FifoDataCount = 0; /* restart */

  Stop_AD5940_SPI(); // Once the test has started, turn off SPI to reduce power
  setRunning();
  return true;
}

bool EChem_Imp::stop() {
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
int32_t EChem_Imp::initAD5940(void) {
  AD5940_HWReset();                                // Hardware reset
  AD5940_Initialize();                             // Platform configuration
  AD5940_ConfigureClock();                         // Step 1 - Configure clock
  AD5940_ConfigureFIFO(FIFOSIZE_4KB, FIFOSRC_DFT); // Step 2 - Configure FIFO and Sequencer
  // AD5940_ConfigureSequencer(SEQMEMSIZE_2KB);
  AD5940_ConfigureInterrupts(AFEINTSRC_DATAFIFOTHRESH); // Step 3 - Configure interrupt controller
  AD5940_ConfigureGPIO();                               // Step 4 - Reconfigure GPIO
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);                  // Enable AFE to enter sleep mode
  AD5940_MeasureLFOSC(&LFOSCFreq);                      // Measure the LFOSC frequency

  return 0;
}

/**
 * @brief Initialize the amperometric test. Call this function every time before starting amperometric test.
 */
AD5940Err EChem_Imp::setupMeasurement(void) {
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

  /* Do RTIA calibration */
  AD5940_CalibrateHSRTIA();

  /* Now Reconfigure FIFO after Rtia cal for CA measurements*/
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE); /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
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
  AD5940_WriteReg(REG_AFE_SWMUX, 1 << 3);

  return AD5940ERR_OK;
}

void EChem_Imp::configureWaveformParameters(void) {
  // AFE mode settings
  if (config.SinFreq >= 20000.0)
    config.PwrMod = AFEPWR_HP;
  else
    config.PwrMod = AFEPWR_LP;

  config.DacVoltPP = config.Eac; // convert to peak to peak. If EXCITBUFGAIN * HsDacGain = 2, then the Eac parameter is
                                 // already effectively peak-to-peak, no need to adjust further.
}

/* Generate init sequence for CA. This runs only one time. */
AD5940Err EChem_Imp::generateInitSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;
  float sinFreq;

  AD5940_SEQGenCtrl(bTRUE);             // Start sequence generator here
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); // Init all to disable state

  bool bLPDACandTIANeeded = config.IMP4WIRE && config.ACcoupled;
  AD5940_ConfigureAFEReferences(bLPDACandTIANeeded, bLPDACandTIANeeded, false, false);

  HSLoopCfg_Type hs_loop = {0};
  hs_loop.HsDacCfg.ExcitBufGain = config.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = config.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = config.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL | SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL | SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if (config.SweepCfg.SweepEn == bTRUE) {
    config.SweepCfg.SweepIndex = 0;
    config.FreqofData = config.SweepCfg.SweepStart;
    config.SweepCurrFreq = config.SweepCfg.SweepStart;
    AD5940_SweepNext(&config.SweepCfg, &config.SweepNextFreq);
    sinFreq = config.SweepCurrFreq;
  } else {
    sinFreq = config.SinFreq;
    config.FreqofData = sinFreq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sinFreq, config.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(config.DacVoltPP / 800.0f * 2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  if (bLPDACandTIANeeded) {
    LPLoopCfg_Type lp_loop = {0};

    lp_loop.LpDacCfg.LpdacSel = LPDAC0;
    lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lp_loop.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA;
    lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lp_loop.LpDacCfg.DataRst = bFALSE;
    lp_loop.LpDacCfg.PowerEn = bTRUE;
    lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((1300 - AD5940_MIN_DAC_OUTPUT) / AD5940_6BIT_DAC_1LSB);
    lp_loop.LpDacCfg.DacData12Bit = (int32_t)(lp_loop.LpDacCfg.DacData6Bit * 64); // don't care, not using it

    lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
    lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_HALF;
    lp_loop.LpAmpCfg.LpPaPwrEn = bFALSE;
    lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(6) | LPTIASW(7) | LPTIASW(9);
    AD5940_LPLoopCfgS(&lp_loop);
  }

  DSPCfg_Type dsp_cfg = {0};
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = config.ADCPgaGain;

  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16; /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = config.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = config.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = config.DftNum;
  dsp_cfg.DftCfg.DftSrc = config.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = config.HanWinEn;

  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_HSTIAPWR | AFECTRL_INAMPPWR | AFECTRL_EXTBUFPWR | AFECTRL_WG |
                      AFECTRL_DACREFPWR | AFECTRL_HSDACPWR | AFECTRL_SINC2NOTCH,
                  bTRUE);

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
AD5940Err EChem_Imp::generateMeasSequence(void) {
  AD5940Err error = AD5940ERR_OK;
  uint32_t const* pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = config.DftSrc;
  clks_cal.DataCount = 1L << (config.DftNum + 2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = config.SysClkFreq / config.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); /* wait 250us */
  sw_cfg.Dswitch = config.DswitchSel;
  sw_cfg.Pswitch = config.PswitchSel;
  sw_cfg.Nswitch = config.NswitchSel;
  sw_cfg.Tswitch = config.TswitchSel | SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);

  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE); /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE);                                /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));                                             /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE); /* Stop ADC convert and DFT */

  if (config.IMP4WIRE)
    AD5940_ADCMuxCfgS(config.SenseP, config.SenseN);
  else
    AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_N_NODE);

  AD5940_AFECtrlS(AFECTRL_WG | AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16 * 50));               // delay for signal settling DFT_WAIT
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE); /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));              /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT | AFECTRL_WG | AFECTRL_ADCPWR, bFALSE); /* Stop ADC convert and DFT */

  sw_cfg.Dswitch = SWD_OPEN;
  sw_cfg.Pswitch = SWP_PL | SWP_PL2;
  sw_cfg.Nswitch = SWN_NL | SWN_NL2;
  sw_cfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg); /* Float switches */
  AD5940_EnterSleepS();         /* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  config.MeasSeqCycleCount = AD5940_SEQCycleTime();
  config.MaxODR = 1 / (((config.MeasSeqCycleCount + 10) / 16.0) * 1E-6);
  if (config.SamplingInterval > config.MaxODR) {
    /* We have requested a sampling rate that cannot be achieved with the time it
       takes to acquire a sample.
    */
    config.SamplingInterval = config.MaxODR;
  }

  if (error == AD5940ERR_OK) {
    config.MeasureSeqInfo.SeqId = SEQID_0;
    config.MeasureSeqInfo.SeqRamAddr = config.InitSeqInfo.SeqRamAddr + config.InitSeqInfo.SeqLen;
    config.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    config.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(config.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  } else
    return error; /* Error */
  return AD5940ERR_OK;
}

AD5940Err EChem_Imp::AD5940_CalibrateHSRTIA(void) {
  HSRTIACal_Type hsrtia_cal;

  hsrtia_cal.AdcClkFreq = config.AdcClkFreq;
  hsrtia_cal.ADCSinc2Osr = config.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = config.ADCSinc3Osr;
  hsrtia_cal.bPolarResult = bTRUE; /* We need magnitude and phase here */
  hsrtia_cal.DftCfg.DftNum = config.DftNum;
  hsrtia_cal.DftCfg.DftSrc = config.DftSrc;
  hsrtia_cal.DftCfg.HanWinEn = config.HanWinEn;
  hsrtia_cal.fRcal = config.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = config.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = config.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = config.SysClkFreq;
  hsrtia_cal.fFreq = config.SweepCfg.SweepStart;

  if (config.SweepCfg.SweepEn == bTRUE) {
    uint32_t i;
    config.SweepCfg.SweepIndex = 0; /* Reset index */
    for (i = 0; i < config.SweepCfg.SweepPoints; i++) {
      AD5940_HSRtiaCal(&hsrtia_cal, config.RtiaCalTable[i]);
      dbgInfo(String("Freq: ") + String(hsrtia_cal.fFreq) + String(", RTIA: Mag: ") +
              String(config.RtiaCalTable[i][0]) + String(" Ohm, Phase: ") + String(config.RtiaCalTable[i][1]));
      AD5940_SweepNext(&config.SweepCfg, &hsrtia_cal.fFreq);
    }
    config.SweepCfg.SweepIndex = 0; /* Reset index */
    config.RtiaCurrValue[0] = config.RtiaCalTable[config.SweepCfg.SweepIndex][0];
    config.RtiaCurrValue[1] = config.RtiaCalTable[config.SweepCfg.SweepIndex][1];
  } else {
    hsrtia_cal.fFreq = config.SinFreq;
    AD5940_HSRtiaCal(&hsrtia_cal, config.RtiaCurrValue);
  }
  return AD5940ERR_OK;
}

// Function to handle interrupts
void EChem_Imp::ISR(void) {
  if (!isRunning()) return; // Check that the technique is running

  std::vector<uint32_t> buf;

  // Read the FIFO
  Start_AD5940_SPI();
  if (AD5940_WakeUp(10) > 10)        /* Wakeup AFE by read register, read 10 times at most */
    return;                          /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK); // We need time to read data from FIFO, do not let AD5940 hibernate
  if (AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE) {
    uint32_t numSamples = AD5940_FIFOGetCnt() / 4 * 4;
    buf.resize(numSamples);
    AD5940_FIFORd(buf.data(), numSamples);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    // updateRegisters
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Unlock so sequencer can put AD5940 to sleep */
    AD5940_EnterSleepS();
  }

  /* Calculate next frequency point */
  if (config.SweepCfg.SweepEn == bTRUE) {
    config.FreqofData = config.SweepCurrFreq;
    config.SweepCurrFreq = config.SweepNextFreq;
    config.RtiaCurrValue[0] = config.RtiaCalTable[config.SweepCfg.SweepIndex][0];
    config.RtiaCurrValue[1] = config.RtiaCalTable[config.SweepCfg.SweepIndex][1];
    AD5940_SweepNext(&config.SweepCfg, &config.SweepNextFreq);
  }

  Stop_AD5940_SPI();

  if (!buf.empty()) processAndStoreData(buf.data(), static_cast<uint32_t>(buf.size()));
}

/* Modify registers when AFE wakeup */
AD5940Err EChem_Imp::updateRegisters(void) {
  if (config.NumOfData > 0) {
    config.FifoDataCount += getNumBytesAvailable() / 4;
    if (config.FifoDataCount >= config.NumOfData) {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if (config.StopRequired == bTRUE) {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if (config.SweepCfg.SweepEn) /* Need to set new frequency and set power mode */
    AD5940_WGFreqCtrlS(config.SweepNextFreq, config.SysClkFreq);
  return AD5940ERR_OK;
}

bool EChem_Imp::processAndStoreData(uint32_t* pData, uint32_t numSamples) {
  if (!pData || (numSamples % 4u) != 0u) return false;

  // Convert DFT result to int32_t type
  for (uint32_t i = 0; i < numSamples; i++) {
    pData[i] &= 0x3ffff;      /* @todo option to check ECC */
    if (pData[i] & (1 << 17)) /* Bit17 is sign bit */
      pData[i] |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
  }

  // Cast the data to the appropriate type
  const iImpCar_Type* impData = reinterpret_cast<const iImpCar_Type*>(pData);

  for (uint32_t i = 0; i < numSamples / 4; i++) {
    // Each DFT result has two data in FIFO, real part and imaginary part.
    const iImpCar_Type& curr = impData[2 * i + 0];
    const iImpCar_Type& volt = impData[2 * i + 1];

    // Calculate the magnitude and phase of the voltage and current
    const float vm = std::hypot(static_cast<float>(volt.Real), static_cast<float>(volt.Image));
    const float vp = std::atan2(-static_cast<float>(volt.Image), static_cast<float>(volt.Real));

    const float im = std::hypot(static_cast<float>(curr.Real), static_cast<float>(curr.Image));
    const float ip = std::atan2(-static_cast<float>(curr.Image), static_cast<float>(curr.Real));

    // Input refer the voltage and current to the RTIA calibration values
    fImpPol_Type Imp;
    Imp.Magnitude = vm / im * config.RtiaCurrValue[0];
    Imp.Phase = vp - ip + config.RtiaCurrValue[1];
    push(Imp);
  }

  return true;
}

void EChem_Imp::printResult(void) {
  const float freq = (config.SweepCfg.SweepEn == bTRUE) ? config.FreqofData : config.SinFreq;

  forEach([freq](const fImpPol_Type& imp) {
    Serial.printf("Freq: %.2f [Hz], Mag: %.5f [Ohm], Phase: %.5f [deg]\n", freq, imp.Magnitude,
                  imp.Phase * 180 / MATH_PI);
  });
}
