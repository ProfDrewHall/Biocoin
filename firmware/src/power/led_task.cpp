#include "power/led_task.h"

#include "HWConfig/constants.h"
#include "power/power.h"

#include <Arduino.h>

namespace power {
    static void heartbeatTask(void* pvParameters);
}

void power::startHeartbeatTask() {
  xTaskCreate(heartbeatTask,   // Task function
              "LED Heartbeat", // Task name
              512,             // Stack size (in words)
              nullptr,         // Task parameters
              0,               // Priority (very low)
              nullptr          // Task handle (optional)
  );
}

void power::heartbeatTask(void* pvParameters) {
  while (true) {
    // LED Heartbeat ON
    pinMode(PIN_LED, OUTPUT);             // Turn the pin back to an output (we move to an input to save power)
    digitalWrite(PIN_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(kBlinkOn));

    // LED Heartbeat OFF
    digitalWrite(PIN_LED, LOW);           // Unclear if this is needed, but it is a good practice
    disconnectInputGPIO(PIN_LED);         // Disconnect the GPIO pin to save power
    vTaskDelay(pdMS_TO_TICKS(kBlinkOff));
  }
}
