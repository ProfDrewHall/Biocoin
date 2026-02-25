/**
 * @file echem_temp.h
 * @brief Temperature measurement technique interface and configuration.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/core/sensor.h"

#include <Arduino.h>

namespace sensor {
  constexpr uint32_t kTempMinFifoThreshold = 1u;
  constexpr uint32_t kTempMaxFifoThreshold = 1023u;
  constexpr uint32_t kTempWakeSleepTicks = 3u;
  constexpr uint32_t kTempWakeAdjustTicks = 5u;
  constexpr uint8_t kTempChannelVafe4 = ADCMUXP_VAFE4;
  constexpr uint8_t kTempChannelVafe1 = ADCMUXP_VAFE1;
  constexpr uint8_t kTempChannelAin2 = ADCMUXP_AIN2;
  constexpr uint32_t kTempSeqRefSettleUs = 250u;
  constexpr uint32_t kTempSeqPostAdcDelayClks = 20u;

  struct TempParameterPayload {
    float samplingInterval;
    float processingInterval;
    uint8_t channel;
  } __attribute__((packed));
  static_assert(sizeof(TempParameterPayload) == 9, "TempParameterPayload must remain packed at 9 bytes");

  struct TempConfig {
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
    uint32_t DataFifoSrc; /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2 */
    float ADCRefVolt;     /* Vref value */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
  };

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
    bool loadParameters(const uint8_t* data, uint16_t len);

    /** @brief Handle temperature ISR path, read FIFO values, and queue converted output. */
    void ISR(void);

    /** @brief Print queued temperature samples for debugging. */
    void printResult(void);
    /**
     * @brief Pop serialized temperature sample bytes from queue.
     * @param num_items Maximum number of bytes to retrieve.
     * @return Byte vector containing packed float sample values.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Return number of queued sample bytes available for BLE TX. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /** @brief Initialize temperature configuration defaults. */
    void initDefaults(void);

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
    AD5940Err generateMeasurementSequence(void);

    /**
     * @brief Convert raw temperature FIFO words to output values and enqueue.
     * @param pData Pointer to raw FIFO words.
     * @param numSamples Number of words in @p pData.
     * @return True when processing succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t numSamples);
    /**
     * @brief Validate host temperature payload fields before applying runtime settings.
     * @param params Parsed temperature payload from host.
     * @param reason Output reason string on validation failure.
     * @return True when payload values are valid for temperature runtime.
     */
    bool validateParameters(const TempParameterPayload& params, String* reason) const;
    /**
     * @brief Build wakeup timer configuration from active temperature sampling parameters.
     * @param cfg Output wakeup timer configuration.
     * @param reason Output reason string on failure.
     * @return True when wakeup configuration is valid and ready to apply.
     */
    bool buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const;
    /**
     * @brief Shared cleanup path for temperature start failures after power-on.
     * @param reason Human-readable failure reason for debug logs.
     */
    void cleanupStartFailure(const String& reason) const;

    TempConfig config;
    TempParameterPayload hostParams;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor

