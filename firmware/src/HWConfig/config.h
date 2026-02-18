// Module: Runtime/build configuration constants.
// Purpose: Centralize firmware versioning and subsystem tunables.

#pragma once

#include <cstdint>

#define BIOCOIN_FW_VERSION "1.1.1" // Keep in sync with release tags/app compatibility.

constexpr char kFirmwareVersion[] = BIOCOIN_FW_VERSION;
constexpr char kWelcomeMessage[] = "Biocoin v" BIOCOIN_FW_VERSION;

namespace battery {
  constexpr uint32_t kBatteryUpdateRate = 5 * 60 * 1000; // Update rate [ms] for battery voltage
  constexpr uint8_t kNumADCSamplesToAverage = 100;       // Number of samples to average per battery reading
  constexpr float kADCDividerIdeal = 4.99e6f / (4.99e6f + 10e6f);
  constexpr float kADCDividerComp = 3.00400804f; // Inverse of the divider -- measured
} // namespace battery

namespace power {
  constexpr uint32_t kBlinkOn = 50;                   // Heartbeat LED on time [ms]
  constexpr uint32_t kBlinkOff = (5 * 60 * 1000) - 50; // Heartbeat LED off time [ms]
} // namespace power

namespace bluetooth {
  // Connection/advertising defaults tuned for BLE throughput and mobile compatibility.
  constexpr int8_t kTxPower = -4;
  constexpr uint16_t kMTURequest = 247;
  constexpr float kAdvSlowRate = 200;
  constexpr float kAdvFastRate = 30;
  constexpr uint16_t kAdvFastTimeout = 30;
  constexpr float kConnectionInterval = 100;
  constexpr float kConnectionIntervalEff = 4000;
  constexpr float kSupervisorTimeout = 31000;

  constexpr char kManufacturer[] = "UCSD BioEE Group";
  constexpr char kModel[] = "Biocoin v" BIOCOIN_FW_VERSION;
  constexpr char kFirmwareRev[] = BIOCOIN_FW_VERSION;

  constexpr uint8_t kUUIDService[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                      0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00};

  constexpr uint8_t kUUIDChrStatus[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                        0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00};

  constexpr uint8_t kUUIDChrDeviceName[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                            0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00};

  constexpr uint8_t kUUIDChrSensorCtrl[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                            0xDE, 0xEF, 0x12, 0x12, 0x28, 0x15, 0x00, 0x00};

  constexpr uint8_t kUUIDChrSensorData[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                            0xDE, 0xEF, 0x12, 0x12, 0x29, 0x15, 0x00, 0x00};

  constexpr uint8_t kUUIDChrSensorParams[] = {0xAA, 0x93, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
                                              0xDE, 0xEF, 0x12, 0x12, 0x2A, 0x15, 0x00, 0x00};
} // namespace bluetooth

namespace sensor {
  constexpr float kmVperLSB = 0.14648438f;       // 2.4V ADC range and 14-bit ADC resolution = 3000mV/16384
  constexpr uint8_t kNumADCSamplesToAverage = 1; // Number of samples to average per current reading
} // namespace sensor

namespace storage {
  constexpr const char* kDefaultName = "BioCoin"; // Must be less than BLE_GAP_DEVNAME_MAX_LEN
} // namespace storage
