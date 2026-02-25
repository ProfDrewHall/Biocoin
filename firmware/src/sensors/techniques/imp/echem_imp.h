/**
 * @file echem_imp.h
 * @brief Impedance spectroscopy (EIS/IMP) technique interface and configuration.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/core/sensor.h"

#include <Arduino.h>

namespace sensor {
  constexpr uint32_t kIMPFifoGranularity = 4u;
  constexpr uint32_t kIMPMinFifoThreshold = 4u;
  constexpr uint32_t kIMPMaxFifoThreshold = 1020u;
  constexpr uint32_t kIMPWakeSleepTicks = 1u;
  constexpr uint32_t kIMPWakeAdjustTicks = 3u;
  constexpr uint32_t kIMPSeqRefSettleUs = 250u;
  constexpr uint32_t kIMPMaxSweepPoints = 100u; /* Buffer depth for RTIA sweep calibration table. */
  struct IMPParameterPayload {
    float samplingInterval;
    float processingInterval;
    uint8_t fourWire;
    uint8_t acCoupled;
    float maxCurrent;
    float Eac;
    float frequency;
  } __attribute__((packed));
  static_assert(sizeof(IMPParameterPayload) == 22, "IMPParameterPayload must remain packed at 22 bytes");

  struct ImpConfig {
    /* Common configuration fields. */
    BoolFlag hasValidParameters; /* True after a valid host payload is loaded. */
    uint32_t SeqStartAddr; /* Initialization sequence start address in AD5940 SRAM. */

    /* Application related parameters */
    float SysClkFreq;       /* The real frequency of system clock */
    float AdcClkFreq;       /* The real frequency of ADC clock */
    uint32_t FifoThresh;    /* FIFO threshold. Should be N*4 */
    float SamplingIntervalS; /* Sampling interval used to configure wakeup timer period [s]. */
    float ProcessingIntervalS; /* Host requested processing interval used to derive FIFO threshold [s]. */
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
    float RtiaCalTable[kIMPMaxSweepPoints][2]; /* Calibrated Rtia value table */
    float FreqofData;                       /* The frequency of latest data sampled */

    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type MeasureSeqInfo;
    BoolFlag StopRequired;  /* After FIFO is ready, stop the measurement sequence */
    uint32_t FifoDataCount; /* Count how many times impedance have been measured */

    uint32_t MeasSeqCycleCount; /* How long the measurement sequence will take */
    float MaxODR;               /* Max ODR for sampling in this config */

    BoolFlag imp4Wire;  /* True for 4-wire impedance measurement, false for 2-wire measurement. */
    BoolFlag acCoupled; /* True when measurement pins are AC-coupled into AD5940. */

  };

  class EChem_Imp : public Sensor, public SensorQueue<fImpPol_Type> {
  public:
    /** @brief Construct an impedance (EIS) technique instance with default sweep/AFE settings. */
    EChem_Imp();

    /**
     * @brief Start impedance measurement by configuring AD5940 and enabling measurement sequence timing.
     * @return True when run starts successfully.
     */
    bool start(void);
    /**
     * @brief Stop impedance measurement and disable active sequencing.
     * @return True when stop operation completes.
     */
    bool stop(void);
    /**
     * @brief Parse and apply host-provided impedance parameter payload.
     * @param data Packed impedance parameters (without technique selector).
     * @param len Payload length in bytes.
     * @return True when parameters are valid and configuration is updated.
     */
    bool loadParameters(const uint8_t* data, uint16_t len);

    /** @brief Handle measurement interrupts, read FIFO content, and update sweep state. */
    void ISR(void);

    /** @brief Print queued impedance result samples for debug. */
    void printResult(void);
    /**
     * @brief Pop serialized impedance samples from queue.
     * @param num_items Maximum byte count to pop.
     * @return Byte vector containing packed `fImpPol_Type` samples.
     */
    std::vector<uint8_t> getData(size_t num_items) override { return SensorQueue<fImpPol_Type>::popBytes(num_items); }
    /** @brief Return queued impedance bytes currently available for TX. */
    size_t getNumBytesAvailable(void) const override { return SensorQueue<fImpPol_Type>::size(); }

  private:
    /** @brief Initialize IMP configuration defaults. */
    void initDefaults(void);

    /**
     * @brief Initialize AD5940 baseline hardware/clock/FIFO resources for impedance mode.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Configure sequences, RTIA calibration data, and FIFO for impedance acquisition.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);
    /** @brief Compute waveform/sweep parameters from active impedance settings. */
    void configureWaveformParameters(void);
    /**
     * @brief Calibrate high-speed RTIA path used for impedance measurement.
     * @return AD5940 status code.
     */
    AD5940Err AD5940_CalibrateHSRTIA(void);
    /**
     * @brief Update AFE registers at runtime and enforce stop conditions.
     * @return AD5940 status code.
     */
    AD5940Err updateRegisters(void);

    /**
     * @brief Generate one-time impedance initialization sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateInitSequence(void);
    /**
     * @brief Generate recurring impedance measurement sequence.
     * @return AD5940 status code.
     */
    AD5940Err generateMeasurementSequence(void);

    /**
     * @brief Convert raw FIFO words into impedance-polar output samples and enqueue them.
     * @param pData Raw FIFO data pointer.
     * @param numSamples Number of 32-bit words in @p pData.
     * @return True when processing succeeds.
     */
    bool processAndStoreData(uint32_t* pData, uint32_t numSamples);
    /**
     * @brief Validate host impedance payload fields before applying runtime settings.
     * @param params Parsed impedance payload from host.
     * @param reason Output reason string on validation failure.
     * @return True when payload values are valid for impedance runtime.
     */
    bool validateParameters(const IMPParameterPayload& params, String* reason) const;
    /**
     * @brief Build wakeup timer configuration from active impedance sampling parameters.
     * @param cfg Output wakeup timer configuration.
     * @param reason Output reason string on failure.
     * @return True when wakeup configuration is valid and ready to apply.
     */
    bool buildWakeupTimerConfig(WUPTCfg_Type& cfg, String* reason) const;
    /**
     * @brief Shared cleanup path for impedance start failures after power-on.
     * @param reason Human-readable failure reason for debug logs.
     */
    void cleanupStartFailure(const String& reason) const;

    ImpConfig config;
    IMPParameterPayload hostParams;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor

