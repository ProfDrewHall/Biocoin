/**
 * @file iontophoresis_runtime.cpp
 * @brief Iontophoresis stimulation task runtime helpers.
 */

#include "sensors/techniques/iontophoresis/iontophoresis.h"

#include "HWConfig/config.h"
#include "HWConfig/pins.h"
#include "power/power.h"
#include "sensors/core/sensor_manager.h"
#include "util/debug_log.h"
#include "util/task_sync.h"

namespace sensor {
  constexpr TickType_t kStimulationTaskStopTimeoutTicks = pdMS_TO_TICKS(1000);
}

void sensor::Iontophoresis::startStimulationMonitoringTask() {
  if (stimulationTaskHandle == nullptr) {
    BaseType_t rc = xTaskCreate(stimulationTask,               // Task function
                                "Stimulation Current Monitor", // Task name
                                512,                           // Stack size (words)
                                this,                          // Task parameters
                                0,                             // Priority
                                &stimulationTaskHandle         // Save handle
    );
    if (rc != pdPASS) {
      stimulationTaskHandle = nullptr;
      dbgError("Failed to start stimulation monitor task");
    }
  }
}

void sensor::Iontophoresis::stopStimulationMonitoringTask() {
  if (stimulationTaskHandle == nullptr) return;
  if (xTaskGetCurrentTaskHandle() == stimulationTaskHandle) return;

  if (!taskSync::requestStopAndWait(stimulationTaskHandle, kStimulationTaskStopTimeoutTicks))
    dbgWarn("Stimulation monitor task stop timed out");
}

void sensor::Iontophoresis::stimulationTask(void* pvParameters) {
  auto* self = static_cast<Iontophoresis*>(pvParameters);
  if (!self) {
    dbgError("stimulationTask received nullptr parameters!");
    vTaskDelete(nullptr);
  }

  auto& config = self->config;

  while (true) {
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;

    // The ADC is enabled inside the analogRead() function
    // NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;        // Turn on the ADC
    power::reconnectInputGPIO(PIN_CURRENT_SENSE_OUT, power::PullConfig::Disabled); // Turn back on the pin

    // Configure the ADC
    analogReference(AR_INTERNAL_2_4);                        // Set the analog reference (default = 3.6V)
    analogSampleTime(SAADC_CH_CONFIG_TACQ_40us);             // Set the ADC Sample and Hold Acquisition time
    analogOversampling(SAADC_OVERSAMPLE_OVERSAMPLE_Over16x); // Set the ADC to Oversample and perform sample averaging:
                                                             // averages 2^OVERSAMPLE samples per reading
    analogCalibrateOffset();
    analogReadResolution(14); // Set the resolution, can be 8, 10, 12 or 14

    // Get the raw ADC value and average the readings
    uint32_t total = 0;
    for (uint8_t i = 0; i < kNumADCSamplesToAverage; i++)
      total += analogRead(PIN_CURRENT_SENSE_OUT);

    // Revert the ADC to default settings
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;  // Turn off the ADC
    power::disconnectInputGPIO(PIN_CURRENT_SENSE_OUT); // Float the pin to save power

    // Calculate the current [uA]
    float avgADCReading = static_cast<float>(total) / kNumADCSamplesToAverage; // Average the reading
    float current = avgADCReading * kmVperLSB / config.Rsense / config.Av_CSA * 1000;

    dbgInfo(String("Measured current (uA): ") + current);

    // check if current exceeds maximum allowed value for safety reasons
    if (current > config.maxCurrent) {
      self->stop();
      updateStatus(TestState::CURRENT_LIMIT_EXCEEDED);
      dbgError("Current threshold exceeded! Shutting down iontophoresis\n");
      break;
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(static_cast<uint32_t>(config.samplingInterval * 1000.0f))) > 0) break;
  }

  self->stimulationTaskHandle = nullptr;
  vTaskDelete(nullptr);
}
