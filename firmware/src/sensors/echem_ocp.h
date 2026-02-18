/**
 * @file echem_ocp.h
 * @brief Open-circuit potential (OCP) technique interface and configuration.
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
    uint32_t DataFifoSrc; /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2*/

    float ADCRefVolt; /*Vref value */
    float Vzero; /* Voltage on SE0 pin and Vzero*/
    float Vbias; /* Sensor bias voltage = VRE0 - VSE0 */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
  } OCPConfig_Type;

  class EChem_OCP : public Sensor, public SensorQueue<float> {
  public:
    /** @brief Construct an Open Circuit Potential (OCP) technique instance. */
    EChem_OCP();

    /**
     * @brief Start OCP sampling by configuring AD5940 and scheduling periodic measurement sequence.
     * @return True when acquisition starts successfully.
     */
    bool start(void);
    /**
     * @brief Stop OCP sampling and shut down measurement resources.
     * @return True when stop operation completes.
     */
    bool stop(void);
    /**
     * @brief Parse and apply OCP parameter payload sent by host.
     * @param data Packed OCP parameter bytes.
     * @param len Number of bytes in @p data.
     * @return True when payload is valid and parameters are applied.
     */
    bool loadParameters(uint8_t* data, uint16_t len);

    /** @brief Handle OCP interrupt workflow, read FIFO data, and queue converted values. */
    void ISR(void);

    /** @brief Print queued OCP values for serial debug output. */
    void printResult(void);
    /** @brief Compatibility hook for shared interface (data handling occurs in ISR path). */
    void processData(void);
    /**
     * @brief Pop serialized OCP sample bytes from queue.
     * @param num_items Maximum number of bytes to fetch.
     * @return Byte vector of packed float samples.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Return count of queued sample bytes pending transmission. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /**
     * @brief Initialize AD5940 blocks required for OCP mode.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Configure OCP runtime resources including sequencer and FIFO.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);

    /**
     * @brief Generate one-time OCP initialization sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateInitSequence(void);
    /**
     * @brief Generate periodic OCP measurement sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateMeasSequence(void);

    /**
     * @brief Convert raw OCP FIFO words into output samples and queue them.
     * @param pData Raw AD5940 FIFO words.
     * @param num_samples Number of samples in @p pData.
     * @return True when conversion/enqueue succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    OCPConfig_Type config;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor
