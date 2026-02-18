#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/Sensor.h"

#include <vector>

namespace sensor {

  typedef struct {
    BoolFlag bParaChanged; /* Indicates that the parameters have been updated and sequence needs to be regenerated */

    float samplingInterval; /* decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and
                               sample frequency decides the maxim interval. */
    float stimCurrent;      // [uA]
    float maxCurrent;       // [uA]
    uint16_t Rsense;        // Rsense resistor value in [ohms]
    uint16_t Av_CSA;        // Current sense amplifier gain in [V/V]

  } IontophoresisConfig_Type;

  class Iontophoresis : public Sensor {
  public:
    Iontophoresis();

    // Control functions
    bool start(void);
    bool stop(void);
    bool loadParameters(uint8_t* data, uint16_t len);

    // Interrupt service routine
    void ISR(void){};

    // Data processing and retrieval
    void printResult(void){};
    void processData(void){};
    std::vector<uint8_t> getData(size_t num_items) override { return std::vector<uint8_t>(); }
    size_t getNumBytesAvailable(void) const override { return 0; }

  private:
    // AD5940 configuration
    int32_t initAD5940(void);
    AD5940Err setupMeasurement(void);
    AD5940Err configureDAC(void);

    // Current monitoring tasks
    void startStimulationMonitoringTask();
    void stopStimulationMonitoringTask();
    static void stimulationTask(void* pvParameters);

    TaskHandle_t stimulationTaskHandle;

    IontophoresisConfig_Type config;

    float LFOSCFreq;
  };

} // namespace sensor
