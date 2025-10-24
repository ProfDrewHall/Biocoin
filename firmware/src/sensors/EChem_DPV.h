#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/sensor.h"

#include <queue>


namespace sensor {

  typedef struct {
    /* Common configurations for all kinds of Application. */
    BoolFlag bParaChanged; /* Indicates that the parameters have been updated and sequence needs to be regenerated */
    uint32_t SeqStartAddr; /* Initialaztion sequence start address in SRAM of AD5940  */
    uint32_t MaxSeqLen;    /* Limit the maximum sequence.   */

    /* Application related parameters */
    float SysClkFreq;    /* The real frequency of system clock */
    float AdcClkFreq;    /* The real frequency of ADC clock */
    uint32_t FifoThresh; /* FIFO threshold. Should be N*4 */
    float RcalVal;       /* RCVl value in Ohm */
    uint32_t PwrMod;     /* Control Chip power mode(LP/HP) */

    /* Receive path configuration */
    uint32_t AFEBW;      // select from AFEBW_250KHZ, AFEBW_100KHZ, AFEBW_50KHZ
    uint32_t ADCPgaGain; /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal
                            is in range of +-1.5V which is limited by ADC input stage */
    uint8_t ADCSinc3Osr; /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
    uint8_t ADCSinc2Osr; /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
    uint32_t DataFifoSrc;      /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2*/
    uint32_t LptiaRtiaSel;     /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K,
                                  RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
    uint32_t LpTiaRf;          /* Rfilter select */
    uint32_t LpTiaRl;          /* SE0 Rload select */
    fImpPol_Type RtiaCalValue; /* Calibrated Rtia value */
    BoolFlag ExtRtia;          /* Use internal or external Rtia */

    float Estart;           //[mv]
    float Estop;            //[mv]
    float RampStartVolt;    /**< The start voltage of ramp signal in mV */
    float RampPeakVolt;     /**< The maximum or minimum voltage of ramp in mV */
    float VzeroStart;       /**< The start voltage of Vzero in mV. Set it to 2400mV by default */
    float VzeroPeak;        /**< The peak voltage of Vzero in mV. Set it to 200mV by default */
    uint32_t StepNumber;    /**< Total number of steps. Limited to 4095. */
    float PulsePeriod;      /**< period of square wave */
    float PulseWidth;       /**< width of square wave pulse*/
    float Epulse;           /**< Set amplitude of square wave */
    float Estep;            /**< Ramp increase in mV */
    float SampleDelay_High; /**< The time delay between update DAC and start ADC */
    float SampleDelay_Low;  /**< The time delay between update DAC and start ADC */

    /* LPDAC Config */
    float ADCRefVolt; /* Vref value */
    float ExtRtiaVal; /* External Rtia value if using one */

    SEQInfo_Type ADC_HighPulse_SeqInfo;
    SEQInfo_Type ADC_LowPulse_SeqInfo;
    BoolFlag bFirstDACSeq;   /**< Init DAC sequence */
    SEQInfo_Type DACSeqInfo; /**< The first DAC update sequence info */
    uint32_t CurrStepPos;    /**< Current position */
    float DACCodePerStep;    /**< DAC codes in square waveform */
    float DACCodePerRamp;    /**< DAC codes needed to ramp increment */
    float CurrRampCode;      /**<  */
    uint32_t CurrVzeroCode;
    BoolFlag bSqrWaveHiLevel; /**< Flag to indicate square wave high level */
  } DPVConfig_Type;

  enum class DPVRampState : uint8_t {
    Start = 0, // Estart -> Evertex1
    State1,    // Evertex1 -> Evertex2
    State2,    // Evertex2 -> Estart
    Stop       // Ramp is complete
  };

  class EChem_DPV : public Sensor, public SensorQueue<float> {
  public:
    EChem_DPV();

    // Control functions
    bool start(void);
    bool stop(void);
    bool loadParameters(uint8_t* data, uint16_t len);

    // Interrupt service routine
    void ISR(void);

    // Data processing and retrieval
    void printResult(void);
    void processData(void);
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    int32_t initAD5940(void);
    AD5940Err setupMeasurement(void);
    AD5940Err configureLPLoop(void);
    void configureWaveformParameters(void);

    // Sequence generation functions
    AD5940Err generateInitSequence(void);
    AD5940Err generateADCSequenceHigh(void);
    AD5940Err generateADCSequenceLow(void);
    AD5940Err generateDACSequence(void);

    AD5940Err updateRampDACCode(uint32_t* pDACData);

    // Processing functions
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    DPVConfig_Type config;
    uint8_t channel;

    DPVRampState rampState;
    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];

    std::queue<float> data_queue; // Data queue
  };
} // namespace sensor
