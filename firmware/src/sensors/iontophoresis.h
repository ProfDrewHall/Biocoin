/**
 * @file iontophoresis.h
 * @brief Iontophoresis stimulation technique interface and configuration.
 */

#pragma once

#include "drivers/ad5940_hal.h"
#include "sensors/sensor.h"

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
    /** @brief Construct iontophoresis technique instance and initialize defaults. */
    Iontophoresis();

    /**
     * @brief Start iontophoresis stimulation and monitoring loop.
     * @return True when stimulation starts successfully.
     */
    bool start(void);
    /**
     * @brief Stop iontophoresis stimulation and release resources.
     * @return True when stop operation completes.
     */
    bool stop(void);
    /**
     * @brief Parse and apply iontophoresis parameter payload.
     * @param data Packed iontophoresis parameter bytes.
     * @param len Payload length in bytes.
     * @return True when parameters are valid and accepted.
     */
    bool loadParameters(uint8_t* data, uint16_t len);

    // Interrupt service routine (unused for this technique)
    void ISR(void){};

    // Data processing and retrieval (no queued sample stream for this technique)
    void printResult(void){};
    void processData(void){};
    std::vector<uint8_t> getData(size_t num_items) override { return std::vector<uint8_t>(); }
    size_t getNumBytesAvailable(void) const override { return 0; }

  private:
    /**
     * @brief Initialize AD5940 resources required for stimulation DAC control.
     * @return 0 on success.
     */
    int32_t initAD5940(void);
    /**
     * @brief Configure iontophoresis runtime measurement/stimulation settings.
     * @return AD5940 status code.
     */
    AD5940Err setupMeasurement(void);
    /**
     * @brief Program DAC output used to drive configured stimulation current.
     * @return AD5940 status code.
     */
    AD5940Err configureDAC(void);

    /**
     * @brief Start background task that monitors delivered current and safety limits.
     */
    void startStimulationMonitoringTask();
    /**
     * @brief Request stop for stimulation monitoring task.
     */
    void stopStimulationMonitoringTask();
    /**
     * @brief Worker task that periodically checks stimulation current and enforces limits.
     * @param pvParameters Pointer to owning `Iontophoresis` instance.
     */
    static void stimulationTask(void* pvParameters);

    TaskHandle_t stimulationTaskHandle;

    IontophoresisConfig_Type config;

    float LFOSCFreq;
  };

} // namespace sensor
