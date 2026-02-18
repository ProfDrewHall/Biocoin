#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/Sensor.h"

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
    EChem_CA();

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

    // Sequence generation functions
    AD5940Err generateInitSequence(void);
    AD5940Err generateMeasSequence(void);

    // Processing functions
    bool processAndStoreData(uint32_t* pData, uint32_t num_samples);

    CAConfig_Type config;

    uint8_t channel;

    float LFOSCFreq;

    const static uint32_t SEQ_BUFF_SIZE = 128;
    uint32_t seq_buffer[SEQ_BUFF_SIZE];
  };

} // namespace sensor
