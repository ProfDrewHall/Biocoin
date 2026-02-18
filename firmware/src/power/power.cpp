#include "power/power.h"

#include "HWConfig/constants.h"
#include "util/debug_log.h"
#include "drivers/ad5940_hal.h"
#include "power/led_task.h"

#include <nrf_sdm.h>
#include <nrf_soc.h>

constexpr uint32_t kStartupDelay = 50; // Time in ms to let peripherals come online

void power::init() {
  dbgInfo("Initializing pins...");
  initPins();               // Set the pin directions
  powerOffPeripherials();   // Default states
  enableDCDC();             // Turn on the dc-dc converter to save power
  startHeartbeatTask();     // Start the LED heartbeat
}

void power::initPins() {
  pinMode(PIN_LED, OUTPUT); // Heartbeat LED
  pinMode(PIN_VDIV, INPUT); // Battery monitor, has a buffer and resistive divider

  // Buck/boost & analog power control
  pinMode(PIN_BOOST_EN, OUTPUT);
  pinMode(PIN_BUCKBOOST_AUX_EN, OUTPUT);
  pinMode(PIN_BUCKBOOST_ANALOG_EN, OUTPUT);
  pinMode(PIN_AFE_VDD_CTRL, OUTPUT);
  pinMode(PIN_TEMP_VDD_CTRL, OUTPUT);

  // MUX control
  pinMode(PIN_MUX_EN, OUTPUT);
  pinMode(PIN_MUX_A0, OUTPUT);
  pinMode(PIN_MUX_A1, OUTPUT);

  // Iontophoresis control
  pinMode(PIN_IONTOPH_NMOS_CTRL, OUTPUT);
  pinMode(PIN_IONTOPH_PMOS_CTRL, OUTPUT);
  pinMode(PIN_CURRENT_SENSE_OUT, INPUT);

  // AD5940
  pinMode(PIN_AFE_IntPin_GPIO0, INPUT);
  pinMode(PIN_AFE_GPIO1, INPUT);
  pinMode(PIN_AFE_GPIO2, INPUT);
  pinMode(PIN_AFE_GPIO4_AUX, INPUT);
  pinMode(PIN_AFE_RESET, OUTPUT);
  pinMode(PIN_SPI_CS, OUTPUT);
}

void power::enableDCDC() {
  uint8_t softDeviceEnabled;
  sd_softdevice_is_enabled(&softDeviceEnabled);
  if (softDeviceEnabled == 1)
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  else
    NRF_POWER->DCDCEN = 1;
}

// This function sets a GPIO as an input, but explicitly disconnects the input buffer.
// This is necessary to remove the power consumption of the buffer for pins that may have intermediate voltage level
// inputs.
void power::disconnectInputGPIO(uint32_t pin) {
  pin = g_ADigitalPinMap[pin];
  NRF_GPIO_Type* port = nrf_gpio_pin_port_decode(&pin);
  port->PIN_CNF[pin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void power::reconnectInputGPIO(uint32_t pin, PullConfig pull = PullConfig::Disabled) {
  pin = g_ADigitalPinMap[pin];
  NRF_GPIO_Type* port = nrf_gpio_pin_port_decode(&pin); // Convert Arduino pin to NRF pin number

  uint32_t pull_config;
  switch (pull) {
  case PullConfig::Pullup:
    pull_config = GPIO_PIN_CNF_PULL_Pullup;
    break;
  case PullConfig::Pulldown:
    pull_config = GPIO_PIN_CNF_PULL_Pulldown;
    break;
  case PullConfig::Disabled:
  default:
    pull_config = GPIO_PIN_CNF_PULL_Disabled;
    break;
  }

  port->PIN_CNF[pin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                       ((uint32_t)pull_config << GPIO_PIN_CNF_PULL_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

// Assume transitioning from peripherialOffState
void power::powerOnAFE(uint8_t muxChannel) {
  digitalWrite(PIN_BUCKBOOST_ANALOG_EN, HIGH); // Turn on the LDO output of the buck/boost converter (Out1)
  digitalWrite(PIN_AFE_VDD_CTRL, LOW);         // AFE on (active low)
  digitalWrite(PIN_TEMP_VDD_CTRL, HIGH);       // Temperature off (active low)
  digitalWrite(PIN_MUX_EN, HIGH);              // Enable the MUX

  //digitalWrite(PIN_IONTOPH_PMOS_CTRL,
  //             HIGH); // Turn off iontopheresis switch -- the power down state assumes LDO is off to save power

  setAFEChannel(muxChannel);

  delay(kStartupDelay); // Needed to let the chips startup before we do anything
}

// Assume transitioning from peripherialOffState
void power::powerOnTempSensor() {
  digitalWrite(PIN_BUCKBOOST_ANALOG_EN, HIGH); // Turn on the LDO output of the buck/boost converter (Out1)
  digitalWrite(PIN_TEMP_VDD_CTRL, LOW);        // Turn on the temp load switch (active low)
  digitalWrite(PIN_AFE_VDD_CTRL, LOW);         // Turn on the AFE load switch (active low)

  //digitalWrite(PIN_IONTOPH_PMOS_CTRL,
  //             HIGH); // Turn off iontopheresis switch -- the power down state assumes LDO is off to save power

  delay(kStartupDelay); // Needed to let the chips startup before we do anything
}

void power::powerOnIontophoresis() {
  digitalWrite(PIN_BOOST_EN, HIGH);            // Boost converter for 20V supply
  digitalWrite(PIN_BUCKBOOST_AUX_EN, HIGH);    // Current monitoring amplifier, separate supply
  digitalWrite(PIN_BUCKBOOST_ANALOG_EN, HIGH); // Turn on the LDO output of the buck/boost converter (Out1)
  digitalWrite(PIN_AFE_VDD_CTRL, LOW);         // AFE on (active low) -- Needed for the DAC
  digitalWrite(PIN_TEMP_VDD_CTRL, HIGH);       // Temperature off (active low)

  digitalWrite(PIN_IONTOPH_NMOS_CTRL, HIGH); // Connect electrode to AFE
  digitalWrite(PIN_IONTOPH_PMOS_CTRL, LOW);  // Connect electrode to AFE (active low)

  delay(kStartupDelay); // Needed to let the chips startup before we do anything
}

void power::powerOffPeripherials() {
  digitalWrite(PIN_BUCKBOOST_ANALOG_EN, LOW); // Turn off the LDO output of the buck/boost converter (Out1)
  digitalWrite(PIN_BOOST_EN, LOW);            // Turn off the 20V boost converter
  digitalWrite(PIN_BUCKBOOST_AUX_EN, LOW);    // Turn off the current monitoring amplifier

  digitalWrite(PIN_TEMP_VDD_CTRL, LOW);       // Turn on the temp load switch (active low) -- Note this is behind the LDO, so
                                              // it is "on" but this reduces pin power
  digitalWrite(PIN_AFE_VDD_CTRL, LOW);        // Turn on the AFE load switch (active low) -- Note this is behind the LDO, so it
                                              // is "on" but this reduces pin power

  digitalWrite(PIN_IONTOPH_NMOS_CTRL, LOW);   // Disable iontopheresis isolation switches
  digitalWrite(PIN_IONTOPH_PMOS_CTRL, LOW);   // NOTE: This is LOW ("on"), but behind the LDO so we keep the state low to
                                              // reduce power. If the LDO is powered, we need to turn this off

  disconnectInputGPIO(PIN_AFE_IntPin_GPIO0); // Enabled when needed, saves power
  disconnectInputGPIO(PIN_CURRENT_SENSE_OUT);
  disconnectInputGPIO(PIN_AFE_GPIO1);
  disconnectInputGPIO(PIN_AFE_GPIO2);
  disconnectInputGPIO(PIN_AFE_GPIO4_AUX);

  digitalWrite(PIN_MUX_EN, LOW);              // Turn off the mux
  digitalWrite(PIN_MUX_A0, LOW);
  digitalWrite(PIN_MUX_A1, LOW);
  digitalWrite(PIN_AFE_RESET, LOW);
  digitalWrite(PIN_SPI_CS, LOW);
}
