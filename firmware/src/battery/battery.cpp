/**
 * @file battery.cpp
 * @brief Battery module initialization and service orchestration.
 */

#include "battery/battery.h"

#include "HWConfig/constants.h"
#include "battery/battery_task.h"
#include "power/power.h"
#include "util/debug_log.h"

#include <Arduino.h>
#include <cmath>

constexpr float kADCRefVoltage = 1.8f;                                                 // Reference voltage in volts
constexpr int kADCResolution = 16383;                                                  // 14-bit ADC resolution
constexpr float kADCScaleFactor = kADCRefVoltage / static_cast<float>(kADCResolution); // Code to Volts
namespace {
  float sampleBatteryVoltage(uint8_t numToAverage) {
    if (numToAverage == 0) numToAverage = 1; // Guard against divide by 0

    power::reconnectInputGPIO(PIN_VDIV, power::PullConfig::Disabled); // Turn back on the pin

    // Configure the ADC
    analogReference(AR_INTERNAL_1_8);                        // Set the analog reference (default = 3.6V)
    analogSampleTime(SAADC_CH_CONFIG_TACQ_40us);             // Set the ADC sample and hold acquisition time
    analogOversampling(SAADC_OVERSAMPLE_OVERSAMPLE_Over16x); // averages 2^OVERSAMPLE samples per reading
    analogReadResolution(14);                                // 8, 10, 12 or 14
    analogRead(PIN_VDIV);
    delay(10); // Let ADC front-end settle before accumulating samples

    float totalVoltage = 0.0f;
    uint8_t validSamples = 0;
    for (uint8_t i = 0; i < numToAverage; i++) {
      const uint16_t adcReading = analogRead(PIN_VDIV);
      const float sampleVoltage = static_cast<float>(adcReading) * kADCScaleFactor * battery::kADCDividerComp;

      if (sampleVoltage < battery::kMinValidBatteryVoltageV || sampleVoltage > battery::kMaxValidBatteryVoltageV) {
        dbgWarn(String("Battery sample out of range, ignoring: ") + sampleVoltage + String(" V"));
        continue;
      }

      totalVoltage += sampleVoltage;
      validSamples++;
    }

    // Revert ADC to default low-power state.
    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
    power::disconnectInputGPIO(PIN_VDIV);

    if (validSamples == 0) {
      dbgWarn("All battery samples were out of range; returning 0V");
      return 0.0f;
    }

    return totalVoltage / static_cast<float>(validSamples);
  }
} // namespace

void battery::init() {
  dbgInfo("Initializing battery monitoring...");
  power::disconnectInputGPIO(PIN_VDIV); // Float the pin to save power
}

void battery::start() {
  startBatteryTask();
}

void battery::stop() {
  stopBatteryTask();
}

float battery::readBatteryVoltageV(uint8_t numToAverage) {
  const float voltage = sampleBatteryVoltage(numToAverage);
  dbgInfo(String("Battery voltage raw read, Vbatt = ") + voltage + String(" V"));
  return voltage;
}

uint8_t battery::readBatteryPercent(uint8_t numToAverage) {
  const float voltage = readBatteryVoltageV(numToAverage);

  // Map it back to %
  const uint8_t batteryPercentage = voltageToPercent(voltage);

  dbgInfo(String("Battery voltage measured, Vbatt = ") + voltage + String(" V, ") + batteryPercentage + String("%"));
  return batteryPercentage;
}

// Map the voltage into a percentage --- 0->100
uint8_t battery::voltageToPercent(float voltage) {
  float percent = 100.0f / (1.0f + std::exp(-kBatteryFitSlope * (voltage - kBatteryFitMidVoltage)));
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  return static_cast<uint8_t>(percent + 0.5f); // Rounded to nearest integer
}
