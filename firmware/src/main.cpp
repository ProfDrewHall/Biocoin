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
#include "power/power.h"
#include "sensors/sensor_manager.h"
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
  while (!Serial)
    delay(10); // Wait for a serial connection if we are debugging

  delay(1000); // Wait for the serial monitor to start (important)

  dbgInfo(kWelcomeMessage);
  dbgInfo("Version info:");
  dbgPrintVersion();
  dbgPrintResetReason();
#endif

  // Initialize the subsystems
  power::init();
  storage::init();
  battery::init();
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
