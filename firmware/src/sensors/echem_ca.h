/**
 * @file echem_ca.h
 * @brief Chronoamperometry (CA) sensor technique interface and configuration.
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

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingInterval; /* decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and
                               sample frequency decides the maxim interval. */
    float RcalVal;          /* Rcal value in Ohm */
    uint32_t PwrMod;        /* Control Chip power mode(LP/HP) */

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

    /* LPDAC Config */
    float Vzero;      /* Voltage on SE0 pin and Vzero*/
    float SensorBias; /* Sensor bias voltage = VRE0 - VSE0 */
    float ADCRefVolt; /*Vref value */
    float ExtRtiaVal; /* External Rtia value if using one */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
  } CAConfig_Type;

  class EChem_CA : public Sensor, public SensorQueue<float> {
  public:
    /** @brief Construct a Chronoamperometry (CA) technique instance with default AD5940 settings. */
    EChem_CA();

    /**
     * @brief Start a CA run by powering hardware, configuring AD5940, and enabling sequencer timing.
     * @return True if the technique enters running state.
     */
    bool start(void);
    /**
     * @brief Stop a CA run and return the board/peripherals to low-power idle state.
     * @return True if stop/shutdown sequence completes.
     */
    bool stop(void);
    /**
     * @brief Parse and apply host-provided CA parameters from BLE payload bytes.
     * @param data Packed CA parameter payload (without technique selector byte).
     * @param len Length of @p data in bytes.
     * @return True when payload is valid and parameters are applied.
     */
    bool loadParameters(uint8_t* data, uint16_t len);

    /** @brief Handle AD5940 interrupt events, drain FIFO data, and enqueue converted samples. */
    void ISR(void);

    /** @brief Print queued CA current samples to serial debug output. */
    void printResult(void);
    /** @brief Compatibility hook for the shared sensor interface (processing occurs in ISR path). */
    void processData(void);
    /**
     * @brief Pop serialized CA sample bytes from the internal queue.
     * @param num_items Maximum number of bytes to pop.
     * @return Byte vector containing packed sample values.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Get total queued sample bytes currently available for transmission. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /**
     * @brief Initialize AD5940 clocks, GPIO, FIFO, sequencer, and interrupt routing for CA.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Build and configure runtime measurement resources before starting CA.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);

    /**
     * @brief Generate one-time CA initialization sequence and write it to sequencer SRAM.
     * @return AD5940 status code.
     */
    AD5940Err generateInitSequence(void);
    /**
     * @brief Generate periodic CA measurement sequence used by wakeup timer.
     * @return AD5940 status code.
     */
    AD5940Err generateMeasSequence(void);

    /**
     * @brief Convert raw FIFO words into current values and enqueue for BLE transport.
     * @param pData Pointer to raw AD5940 FIFO words.
     * @param num_samples Number of 32-bit words in @p pData.
     * @return True when conversion/enqueue succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    CAConfig_Type config;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor
