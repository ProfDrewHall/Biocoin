/**
 * @file main.cpp
 * @brief Firmware entry point and top-level initialization flow.
 */

//////////////////////////////////
//            Headers           //
//////////////////////////////////
#include "HWConfig/constants.h"
#include "battery/battery.h"
#include "bluetooth/bluetooth.h"
#include "digital/digital_io_manager.h"
#include "power/power.h"
#include "sensors/core/sensor_manager.h"
#include "storage/storage.h"
#include "util/debug_log.h"
#include "util/util.h"

#include <Arduino.h>


//////////////////////////////////
//       Initialization         //
//////////////////////////////////
void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200); // The baudrate here does not matter when using USB CDC
  const uint32_t serialWaitStartMs = millis();
  constexpr uint32_t kSerialWaitTimeoutMs = 2000;
  while (!Serial && (millis() - serialWaitStartMs) < kSerialWaitTimeoutMs) {
    delay(10); // Wait briefly for serial in debug builds, but do not block BLE startup.
  }

  if (Serial) delay(1000); // Give the serial monitor time to attach when present.

  dbgInfo(kWelcomeMessage);
  dbgInfo("Version info:");
  dbgPrintVersion();
  dbgPrintResetReason();
#endif

  // Initialize the subsystems
  power::init();
  storage::init();
  battery::init();
  digital::init();
  bluetooth::init();
  sensor::init();

#ifdef DEBUG_MODE
  // dbgPrintDetailedPinStatus();
  // dbgPrintInterrupts();
  dbgInfo("Done initializing... Off to sleep!");
#endif

  suspendLoop(); // This code is event driven -- it does not use the main loop
}

void loop() {}
