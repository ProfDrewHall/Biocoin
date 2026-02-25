/**
 * @file echem_ca.h
 * @brief Chronoamperometry (CA) sensor technique interface and configuration.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/core/sensor.h"

#include <Arduino.h>

namespace sensor {
  /** @brief Maximum host-selectable channel index for CA measurements. */
  constexpr uint8_t kCAMaxChannel = 3u;
  /** @brief Minimum FIFO threshold supported by CA runtime. */
  constexpr uint32_t kCAMinFifoThreshold = 1u;
  /** @brief Maximum FIFO threshold supported by CA runtime for 4KB FIFO mode. */
  constexpr uint32_t kCAMaxFifoThreshold = 1023u;
  /** @brief Wakeup timer sleep ticks for CA (minimum practical value in this flow). */
  constexpr uint32_t kCAWakeSleepTicks = 3u; // 4 - 1
  /** @brief Ticks subtracted from computed wakeup period to account for wakeup overhead. */
  constexpr uint32_t kCAWakeAdjustTicks = 5u; // 4 + 1
  /** @brief Reference-settling delay after ADC power-up, in microseconds converted by sequencer macro. */
  constexpr uint32_t kCASeqRefSettleUs = 250u;
  /** @brief Extra sequencer wait clocks before entering hibernate to ensure FIFO move completion. */
  constexpr uint32_t kCASeqPostAdcDelayClks = 20u;

  /**
   * @brief Packed host payload for CA parameter updates.
   * @details Byte layout must match BLE payload exactly.
   */
  struct CAParameterPayload {
    float samplingInterval;   /**< [s] ADC sample cadence controlled by wake timer period. */
    float processingInterval; /**< [s] MCU FIFO-processing cadence target. */
    float maxCurrent;         /**< [mA] Requested current range hint from host. */
    float pulsePotential;     /**< [mV] Pulse amplitude (sensor bias). */
    uint8_t channel;          /**< AFE mux channel index (0-3). */
  } __attribute__((packed));
  static_assert(sizeof(CAParameterPayload) == 17, "CAParameterPayload must remain packed at 17 bytes");

  struct CAConfig {
    /* Common configuration fields. */
    BoolFlag hasValidParameters; /* True after a valid host payload is loaded. */
    uint32_t SeqStartAddr; /* Initialization sequence start address in AD5940 SRAM. */

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingIntervalS; /* Sampling interval used to configure wakeup timer period [s]. */
    float ProcessingIntervalS; /* Host requested processing interval used to derive FIFO threshold [s]. */
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
  };

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
    bool loadParameters(const uint8_t* data, uint16_t len);

    /** @brief Handle AD5940 interrupt events, drain FIFO data, and enqueue converted samples. */
    void ISR(void);

    /** @brief Print queued CA current samples to serial debug output. */
    void printResult(void);
    /**
     * @brief Pop serialized CA sample bytes from the internal queue.
     * @param num_items Maximum number of bytes to pop.
     * @return Byte vector containing packed sample values.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<float>::popBytes(num_items); }
    /** @brief Get total queued sample bytes currently available for transmission. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<float>::size(); }

  private:
    /** @brief Initialize CA configuration defaults. */
    void initDefaults(void);

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
    AD5940Err generateMeasurementSequence(void);

    /**
     * @brief Convert raw FIFO words into current values and enqueue for BLE transport.
     * @param pData Pointer to raw AD5940 FIFO words.
     * @param numSamples Number of 32-bit words in @p pData.
     * @return True when conversion/enqueue succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t numSamples);
    /**
     * @brief Validate host CA payload fields before applying them to runtime config.
     * @param params Parsed CA payload from host.
     * @param reason Output reason string on validation failure.
     * @return True when parameter set is valid for CA runtime.
     */
    bool validateParameters(const CAParameterPayload& params, String* reason) const;
    /**
     * @brief Build wakeup-timer configuration from current CA runtime settings.
     * @param cfg Output wakeup timer configuration.
     * @param reason Output reason string on failure.
     * @return True when wakeup configuration is valid and ready to apply.
     */
    bool buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const;
    /**
     * @brief Common cleanup path for CA start failures after AFE power-on/SPI init.
     * @param reason Human-readable failure reason for debug logs.
     */
    void cleanupStartFailure(const String& reason) const;

    CAConfig config;
    CAParameterPayload hostParams;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor

