/**
 * @file echem_temp.h
 * @brief Temperature measurement technique interface and configuration.
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
    uint32_t PwrMod;        /* Control Chip power mode(LP/HP) */

    /* Receive path configuration */
    uint32_t AFEBW;      // select from AFEBW_250KHZ, AFEBW_100KHZ, AFEBW_50KHZ
    uint32_t ADCPgaGain; /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal
                            is in range of +-1.5V which is limited by ADC input stage */
    uint8_t ADCSinc3Osr; /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
    uint8_t ADCSinc2Osr; /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
    uint32_t DataFifoSrc;      /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2*/
    float ADCRefVolt; /*Vref value */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
  } TempConfig_Type;

  class EChem_Temp : public Sensor, public SensorQueue<float> {
  public:
    /** @brief Construct a temperature technique instance with default ADC/AFE settings. */
    EChem_Temp();

    /**
     * @brief Start temperature sampling by configuring AD5940 and enabling periodic measurement.
     * @return True when sampling starts successfully.
     */
    bool start(void);
    /**
     * @brief Stop temperature sampling and power down measurement resources.
     * @return True when stop path completes.
     */
    bool stop(void);
    /**
     * @brief Parse and apply temperature parameter payload from host.
     * @param data Packed temperature parameter bytes.
     * @param len Number of bytes in @p data.
     * @return True when parameters are valid and applied.
     */
    bool loadParameters(uint8_t* data, uint16_t len);

    /** @brief Handle temperature ISR path, read FIFO values, and queue converted output. */
    void ISR(void);

    /** @brief Print queued temperature samples for debugging. */
    void printResult(void);
    /** @brief Compatibility hook for shared sensor interface. */
    void processData(void);
    /**
     * @brief Pop serialized temperature sample bytes from queue.
     * @param num_items Maximum number of bytes to retrieve.
     * @return Byte vector containing packed float sample values.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Return number of queued sample bytes available for BLE TX. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /**
     * @brief Initialize AD5940 baseline hardware resources for temperature mode.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Configure sequencer/FIFO runtime settings for temperature sampling.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);

    /**
     * @brief Generate one-time temperature initialization sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateInitSequence(void);
    /**
     * @brief Generate periodic temperature measurement sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateMeasSequence(void);

    /**
     * @brief Convert raw temperature FIFO words to output values and enqueue.
     * @param pData Pointer to raw FIFO words.
     * @param num_samples Number of words in @p pData.
     * @return True when processing succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    TempConfig_Type config;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor
