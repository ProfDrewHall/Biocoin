/**
 * @file echem_cv.h
 * @brief Cyclic voltammetry (CV) sensor technique interface and configuration.
 */

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

    float Estart;   //[mv]
    float Evertex1; //[mv]
    float Evertex2; //[mv]
    float Estep;    //[mv]
    float PulseWidth;

    float RampStartVolt; /**< The start voltage of ramp signal in mV */
    float RampPeakVolt;  /**< The maximum or minimum voltage of ramp in mV */
    float VzeroStart;    /**< The start voltage of Vzero in mV. Set it to 2400mV by default */
    float VzeroPeak;     /**< The peak voltage of Vzero in mV. Set it to 200mV by default */
    float StepNumberSeg1;
    float StepNumberSeg2;
    float StepNumberSeg3;
    uint32_t StepNumber; /**< Total number of steps. Limited to 4095. */
    /* Receive path configuration */
    float SampleDelay; /**< The time delay between update DAC and start ADC */

    /* LPDAC Config */
    float ADCRefVolt; /* Vref value */
    float ExtRtiaVal; /* External Rtia value if using one */

    BoolFlag bTestFinished;
    SEQInfo_Type InitSeqInfo;

    SEQInfo_Type ADCSeqInfo;
    BoolFlag bFirstDACSeq;   /**< Init DAC sequence */
    SEQInfo_Type DACSeqInfo; /**< The first DAC update sequence info */
    uint32_t CurrStepPos;    /**< Current position */
    float DACCodePerStep;    /**<  */
    float CurrRampCode;      /**<  */
    uint32_t CurrVzeroCode;
    BoolFlag bDACCodeInc; /**< Increase DAC code.  */
  } CVConfig_Type;

  enum class RampState : uint8_t {
    Start = 0, // Estart -> Evertex1
    Vertex1,   // Evertex1 -> Evertex2
    Vertex2,   // Evertex2 -> Estart
    Return,    // Finishing ramp
    Stop       // Ramp is complete
  };

  class EChem_CV : public Sensor, public SensorQueue<float> {
  public:
    /** @brief Construct CV technique object with default AD5940 configuration. */
    EChem_CV();

    /**
     * @brief Start the CV measurement sequence.
     * @return True when the run starts successfully.
     */
    bool start(void);
    /**
     * @brief Stop the CV measurement sequence and power down peripherals.
     * @return True when stop/shutdown completes successfully.
     */
    bool stop(void);
    /**
     * @brief Load host-provided CV parameters.
     * @param data Packed payload bytes for CV parameters.
     * @param len Payload length in bytes.
     * @return True when parameters are accepted and applied.
     */
    bool loadParameters(uint8_t* data, uint16_t len);

    /** @brief Handle AD5940 interrupts and move acquired samples into the queue. */
    void ISR(void);

    /** @brief Print queued CV samples for debug output. */
    void printResult(void);
    /** @brief Reserved processing entry point for compatibility with sensor interface patterns. */
    void processData(void);
    /**
     * @brief Pop serialized sample bytes from the queue.
     * @param num_items Maximum number of bytes requested.
     * @return Byte vector containing serialized samples.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Get currently buffered sample bytes available for TX. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /**
     * @brief Initialize AD5940 clocks, FIFO, sequencer, interrupts, and LFOSC calibration.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Configure runtime measurement sequences and FIFO after parameters are loaded.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);
    /** @brief Derive ramp segments and step counts from requested voltage parameters. */
    void configureRampParameters(void);

    /**
     * @brief Generate and store one-time CV initialization sequence in AD5940 SRAM.
     * @return AD5940 status code.
     */
    AD5940Err generateInitSequence(void);
    /**
     * @brief Generate and store recurring ADC acquisition sequence in AD5940 SRAM.
     * @return AD5940 status code.
     */
    AD5940Err generateADCSequence(void);
    /**
     * @brief Generate/update DAC stepping sequences for ramp progression.
     * @return AD5940 status code.
     */
    AD5940Err generateDACSequence(void);

    /**
     * @brief Advance CV ramp state and compute next DAC register word.
     * @param pDACData Output pointer for packed VZERO/VBIAS DAC code.
     * @return AD5940 status code.
     */
    AD5940Err updateRampDACCode(uint32_t* pDACData);

    /**
     * @brief Convert raw ADC/FIFO words to current samples and enqueue them.
     * @param pData Raw FIFO sample data.
     * @param num_samples Number of raw sample words.
     * @return True when processing succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    CVConfig_Type config;
    uint8_t channel;

    RampState rampState; // Current state of the ramp
    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };
} // namespace sensor
