/**
 * @file echem_ocp.h
 * @brief Open-circuit potential (OCP) technique interface and configuration.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/core/sensor.h"

#include <Arduino.h>

namespace sensor {
  constexpr uint32_t kOCPMinFifoThreshold = 1u;
  constexpr uint32_t kOCPMaxFifoThreshold = 1023u;
  constexpr uint32_t kOCPWakeSleepTicks = 3u;
  constexpr uint32_t kOCPWakeAdjustTicks = 5u;
  constexpr uint8_t kOCPChannelAin6 = ADCMUXP_AIN6;
  constexpr uint8_t kOCPChannelVafe3 = ADCMUXP_VAFE3;
  constexpr uint8_t kOCPChannelAin0 = ADCMUXP_AIN0;
  constexpr uint8_t kOCPChannelVafe2 = ADCMUXP_VAFE2;
  constexpr uint32_t kOCPSeqRefSettleUs = 250u;
  constexpr uint32_t kOCPSeqPostAdcDelayClks = 20u;

  struct OCPParameterPayload {
    float samplingInterval;
    float processingInterval;
    uint8_t channel;
  } __attribute__((packed));
  static_assert(sizeof(OCPParameterPayload) == 9, "OCPParameterPayload must remain packed at 9 bytes");

  struct OCPConfig {
    /* Common configuration fields. */
    BoolFlag hasValidParameters; /* True after a valid host payload is loaded. */
    uint32_t SeqStartAddr; /* Initialization sequence start address in AD5940 SRAM. */

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingIntervalS; /* Sampling interval used to configure wakeup timer period [s]. */
    float ProcessingIntervalS; /* Host requested processing interval used to derive FIFO threshold [s]. */
    uint32_t PwrMod;        /* Control Chip power mode(LP/HP) */

    /* Receive path configuration */
    uint32_t AFEBW;      // select from AFEBW_250KHZ, AFEBW_100KHZ, AFEBW_50KHZ
    uint32_t ADCPgaGain; /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal
                            is in range of +-1.5V which is limited by ADC input stage */
    uint8_t ADCSinc3Osr; /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
    uint8_t ADCSinc2Osr; /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
    uint32_t DataFifoSrc; /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2*/

    float ADCRefVolt; /* Vref value */
    float Vzero;      /* Voltage on SE0 pin and Vzero */
    float Vbias;      /* Sensor bias voltage = VRE0 - VSE0 */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
  };

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
    bool loadParameters(const uint8_t* data, uint16_t len);

    /** @brief Handle OCP interrupt workflow, read FIFO data, and queue converted values. */
    void ISR(void);

    /** @brief Print queued OCP values for serial debug output. */
    void printResult(void);
    /**
     * @brief Pop serialized OCP sample bytes from queue.
     * @param num_items Maximum number of bytes to fetch.
     * @return Byte vector of packed float samples.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Return count of queued sample bytes pending transmission. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /** @brief Initialize OCP configuration defaults. */
    void initDefaults(void);

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
    AD5940Err generateMeasurementSequence(void);

    /**
     * @brief Convert raw OCP FIFO words into output samples and queue them.
     * @param pData Raw AD5940 FIFO words.
     * @param numSamples Number of samples in @p pData.
     * @return True when conversion/enqueue succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t numSamples);
    /**
     * @brief Validate host OCP payload fields before applying runtime settings.
     * @param params Parsed OCP payload from host.
     * @param reason Output reason string on validation failure.
     * @return True when the payload values are valid for OCP runtime.
     */
    bool validateParameters(const OCPParameterPayload& params, String* reason) const;
    /**
     * @brief Build wakeup timer configuration from active OCP sampling parameters.
     * @param cfg Output wakeup timer configuration.
     * @param reason Output reason string on failure.
     * @return True when wakeup configuration is valid and ready to apply.
     */
    bool buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const;
    /**
     * @brief Shared cleanup path for OCP start failures after power-on.
     * @param reason Human-readable failure reason for debug logs.
     */
    void cleanupStartFailure(const String& reason) const;

    OCPConfig config;
    OCPParameterPayload hostParams;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor

