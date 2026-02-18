#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/Sensor.h"

#include <queue>

namespace sensor {
#define MAXSWEEP_POINTS 100 /* Need to know how much buffer is needed to save RTIA calibration result */

  typedef struct {
    /* Common configurations for all kinds of Application. */
    BoolFlag bParaChanged; /* Indicate parameters have been set  */
    uint32_t SeqStartAddr; /* Initialaztion sequence start address in SRAM of AD5940  */

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingInterval; /* decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and
                               sample frequency decides the maxim interval. */
    int32_t NumOfData; /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value
                          here. Otherwise, set it to '-1' which means never stop. */
    float RcalVal;     /* Rcal value in Ohm */
    uint32_t PwrMod;   /* Control Chip power mode(LP/HP) */

    /* Receive path configuration */
    uint32_t SenseP; // positive sense pin
    uint32_t SenseN; // negative sense pin

    uint32_t DswitchSel;
    uint32_t PswitchSel;
    uint32_t NswitchSel;
    uint32_t TswitchSel;
    uint32_t HstiaRtiaSel; /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K,
                              RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
    uint32_t CtiaSel;      /* Select CTIA in pF unit from 0 to 31pF */
    uint32_t ExcitBufGain; /* Select from  EXCTBUFGAIN_2, EXCTBUFGAIN_0P25 */
    uint32_t HsDacGain;    /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */
    uint32_t HsDacUpdateRate;
    float DacVoltPP;   /* DAC output voltage in mV peak to peak. Maximum value is 800mVpp. Peak to peak voltage  */
    float SinFreq;     /* Frequency of excitation signal */
    float Eac;         /* Peak amplitude of sine wave [mV]*/
    uint32_t DftNum;   /* DFT number */
    uint32_t DftSrc;   /* DFT Source */
    BoolFlag HanWinEn; /* Enable Hanning window */

    uint32_t AFEBW;      // select from AFEBW_250KHZ, AFEBW_100KHZ, AFEBW_50KHZ
    uint32_t ADCPgaGain; /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal
                            is in range of +-1.5V which is limited by ADC input stage */
    uint8_t ADCSinc3Osr; /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
    uint8_t ADCSinc2Osr; /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */

    /* Sweep Function Control */
    SoftSweepCfg_Type SweepCfg;
    float SweepCurrFreq;
    float SweepNextFreq;
    float RtiaCurrValue[2];                 /* Calibrated Rtia value of current frequency */
    float RtiaCalTable[MAXSWEEP_POINTS][2]; /* Calibrated Rtia Value table */
    float FreqofData;                       /* The frequency of latest data sampled */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
    BoolFlag StopRequired;  /* After FIFO is ready, stop the measurement sequence */
    uint32_t FifoDataCount; /* Count how many times impedance have been measured */

    uint32_t MeasSeqCycleCount; /* How long the measurement sequence will take */
    float MaxODR;               /* Max ODR for sampling in this config */

    BoolFlag IMP4WIRE; // flag to specify whether experiment is 4-wire or 2-wire impedance measurement. 4-wire == bTRUE,
                       // 2-wire == bFALSE
    BoolFlag ACcoupled; // flag to specify if the measurement pins/wires are AC coupled into the AD5940. bTRUE = AC
                        // coupled, bFALSE = DC coupled

  } ImpConfig_Type;

  class EChem_Imp : public Sensor, public SensorQueue<fImpPol_Type> {
  public:
    EChem_Imp();

    // Control functions
    bool start(void);
    bool stop(void);
    bool loadParameters(uint8_t* data, uint16_t len);

    // Interrupt service routine
    void ISR(void);

    // Data processing and retrieval
    void printResult(void);
    void processData(void);
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<fImpPol_Type>::popBytes(num_items); }
    size_t getNumBytesAvailable(void) const override { return SensorQueue<fImpPol_Type>::size(); }

  private:
    int32_t initAD5940(void);
    AD5940Err setupMeasurement(void);
    void configureWaveformParameters(void);
    AD5940Err AD5940_CalibrateHSRTIA(void);
    AD5940Err updateRegisters(void);

    // Sequence generation functions
    AD5940Err generateInitSequence(void);
    AD5940Err generateMeasSequence(void);

    // Processing functions
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    ImpConfig_Type config;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor
