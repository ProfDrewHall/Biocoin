#include "battery/battery.h"

#include "HWConfig/constants.h"
#include "battery/battery_task.h"
#include "util/debug_log.h"
#include "power/power.h"

#include <Arduino.h>

using namespace battery;

constexpr float kADCRefVoltage = 1.8f;                                                 // Reference voltage in volts
constexpr int kADCResolution = 16383;                                                  // 14-bit ADC resolution
constexpr float kADCScaleFactor = kADCRefVoltage / static_cast<float>(kADCResolution); // Code to Volts

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

uint8_t battery::readLevel(uint8_t numToAverage) {
  if(numToAverage == 0) numToAverage = 1;                 // Guard against divide by 0
  
  // The ADC is enabled inside the analogRead() function
  //NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled;        // Turn on the ADC
  power::reconnectInputGPIO(PIN_VDIV, power::PullConfig::Disabled); // Turn back on the pin

  // Configure the ADC
  analogReference(AR_INTERNAL_1_8);                        // Set the analog reference (default = 3.6V)
  analogSampleTime(SAADC_CH_CONFIG_TACQ_40us);             // Set the ADC Sample and Hold Acquisition time
  analogOversampling(SAADC_OVERSAMPLE_OVERSAMPLE_Over16x); // Set the ADC to Oversample and perform sample averaging:
                                                           // averages 2^OVERSAMPLE samples per reading
  analogReadResolution(14);                                // Set the resolution, can be 8, 10, 12 or 14

  // Get the raw ADC value and average the readings
  uint32_t total = 0;
  for (uint8_t i = 0; i < numToAverage; i++)
    total += analogRead(PIN_VDIV);

  // Revert the ADC to default settings
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;         // Turn off the ADC
  power::disconnectInputGPIO(PIN_VDIV);                     // Float the pin to save power

  // Calculate the battery voltage
  float avgADCReading = static_cast<float>(total) / numToAverage;    // Average the reading
  float voltage = avgADCReading * kADCScaleFactor * kADCDividerComp; // Convert the reading back into code

  // Map it back to %
  uint8_t batteryPercentage = mapBatteryLevel(voltage);

  dbgInfo(String("Battery voltage measured, Vbatt = ") + voltage + String(" V, ") + batteryPercentage + String("%"));
  return batteryPercentage;
}

// Map the voltage into a percentage --- 0->100
uint8_t battery::mapBatteryLevel(float voltage) {
  constexpr float kMaxVoltage = 4.20f; // Fully charged, nominal output is 3.7V
  constexpr float kMinVoltage = 3.00f; // Empty/cutoff

  if (voltage >= kMaxVoltage)
    return 100;
  else if (voltage <= kMinVoltage)
    return 0;
  else {
    float percent = (voltage - kMinVoltage) / (kMaxVoltage - kMinVoltage) * 100.0f;
    return static_cast<uint8_t>(percent + 0.5f); // Rounded to nearest integer
  }
}
