#pragma once

#include <cstdint>
#include <Arduino.h>

// Debugging control -- Turn on in PlatformIO.ini
//#define DEBUG_MODE
//#define DEBUG_LEVEL DBG_LEVEL_INFO // Set to NONE, ERROR, WARN, or INFO
// Info is the most verbose, none is the least

//////////////////////////////////
//   Environment Asserts        //
//////////////////////////////////
#ifdef USE_LFXO // Check the board config (oscillator)
// /Users/risabsankarstuff/.platformio/packages/framework-arduinoadafruitnrf52/variants/feather_nrf52840_express/variant.h
#error "This board does not have a 32kHz XTAL!! Uncomment this in variant.h"
#endif
#ifdef EXTERNAL_FLASH_USE_QSPI // Check the external flash
#error "This board does not have an external flash chip!! Uncomment this in variant.h"
#endif
#if (PINS_COUNT != 38) // Used to lookup the pins
// C:\Users\<username>\AppData\Local\Arduino15\packages\adafruit\hardware\nrf52\1.5.0\variants\feather_nrf52840_express
// Uncomment Port 1.11 -1.13, comment out D34, and change PINS_COUNT to 38
// Change NUM_DIGITAL_PINS to 38
#error "Update pins count to enable port 1 pins in variant.h and variant.cpp."
#endif

constexpr const char* kWelcomeMessage = "Biocoin v1.4";

namespace battery {
  constexpr uint32_t kBatteryUpdateRate = 5*60*1000;   // Update rate [ms] for battery voltage -- 15 minutes
  constexpr uint8_t kNumADCSamplesToAverage = 100;      // Number of samples to average per battery reading
  constexpr float kADCDividerIdeal = 4.99e6f / (4.99e6f + 10e6f); // voltage divider on VBAT 4.99M / (4.99M + 10M)
  constexpr float kADCDividerComp = 3.00400804f;        // Inverse of the divider -- measured
} // namespace battery

namespace power {
  constexpr uint32_t kBlinkOn = 50;                     // Heartbeat LED on time [ms]
  constexpr uint32_t kBlinkOff = (5*60*1000) - 50; //9950;                  // Heartbeat LED off time [ms]
} // namespace power

namespace bluetooth {
  constexpr int8_t kTxPower = -4;                      // Tx Power [dBm] (Valid options: -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 )
  constexpr uint16_t kMTURequest = 247;                // Maximum Transmit Unit (MTU) size to request. Starts with 27. iOS Max is 185 (247 in newer versions), Android 247-512. Default is 23. 
                                                       // Note that in the Nordic NRF5 SoftDevice, the maximum MTU size is 247.    
  constexpr float kAdvSlowRate = 200;                  // Time [ms] between advertising events in slow mode, rounded to 0.625ms chunks
  constexpr float kAdvFastRate = 30;                   // Time [ms] between advertising events in fast mode, rounded to 0.625ms chunks
                                                       // For recommended advertising interval, https://developer.apple.com/library/content/qa/qa1931/_index.html   
  constexpr uint16_t kAdvFastTimeout = 30;             // Advertising time [s] in fast mode before going to slow mode
  constexpr float kConnectionInterval = 100;           // Connection interval [ms] when data is ready to be sent
  constexpr float kConnectionIntervalEff = 4000;       // Connection interval [ms] accounting for slave latency (effective interval = conninterval*(1+slavelatency))
                                                       // Must have kConnectionIntervalEff × (1 + slave_latency) ≤ supervision_timeout / 2
  constexpr float kSupervisorTimeout = 31000;          // Maximum time [ms] between "connection intervals" before the connection is terminated. Max is 32s.
  
  constexpr const char* kManufacturer = "UCSD BioEE Group";
  constexpr const char* kModel = "Biocoin v1.4";
  
  const uint8_t kUUIDService[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                  0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00};

  const uint8_t kUUIDChrStatus[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                    0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00};

  const uint8_t kUUIDChrDeviceName[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                        0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00};

  const uint8_t kUUIDChrSensorCtrl[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                        0xDE, 0xEF, 0x12, 0x12, 0x28, 0x15, 0x00, 0x00};

  const uint8_t kUUIDChrSensorData[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                        0xDE, 0xEF, 0x12, 0x12, 0x29, 0x15, 0x00, 0x00};

  const uint8_t kUUIDChrSensorParams[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                          0xDE, 0xEF, 0x12, 0x12, 0x2A, 0x15, 0x00, 0x00};
} // namespace bluetooth

namespace sensor {
  const float kmVperLSB = 0.14648438f; // 2.4V ADC range and 14-bit ADC resolution = 3000mV/16384
    constexpr uint8_t kNumADCSamplesToAverage = 1;      // Number of samples to average per current reading

}

namespace storage {
  constexpr const char* kDefaultName = "BioCoin";     // Must be less than BLE_GAP_DEVNAME_MAX_LEN (248 characters)
} // namespace storage


/******************************************************************************
 * Board configuration Biocoin
 *
 *      Constant                Value                     Meaning
 *****************************************************************************/


#define SYS_CLOCK_FREQ 64000000.0f // Defined system frequency of the MCU

#define AD5940_MAX_DAC_OUTPUT 2400.0 // Max. voltage the 6-Bit and 12-Bit DAC can provide
#define AD5940_MIN_DAC_OUTPUT 200.0 // Min. voltage the 6-Bit and 12-Bit DAC can provide
#define AD5940_6BIT_DAC_1LSB ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 64) // Size of one LSB of the 6-Bit DAC
#define AD5940_12BIT_DAC_1LSB                                                                                          \
  ((AD5940_MAX_DAC_OUTPUT - AD5940_MIN_DAC_OUTPUT) / 4095) // Size of one LSB of the 12-Bit DAC

// define pins
#define PIN_MUX_A0 37
#define PIN_MUX_A1 34
#define PIN_MUX_EN 35
#define PIN_VDIV 20
#define PIN_LED 28

#define PIN_AFE_GPIO1 16
#define PIN_AFE_GPIO2 17
#define PIN_AFE_GPIO3_AUX 12
#define PIN_AFE_GPIO4_AUX 5

#define PIN_IONTOPH_NMOS_CTRL 10
#define PIN_IONTOPH_PMOS_CTRL 6
#define PIN_CURRENT_SENSE_OUT 21
#define PIN_BOOST_EN 13
#define PIN_BUCKBOOST_AUX_EN 8 // CH2
#define PIN_BUCKBOOST_ANALOG_EN 29 // CH1
#define PIN_AFE_VDD_CTRL 9
#define PIN_TEMP_VDD_CTRL 11

#define PIN_UARTRX 1

// for communicating with AD5940
#define PIN_AFE_IntPin_GPIO0 15
#define PIN_AFE_RESET 18
#define PIN_SPI_CS 14
//#define PIN_SPI_MISO 24
