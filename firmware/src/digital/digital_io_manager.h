/**
 * @file digital_io_manager.h
 * @brief Persistent runtime digital I/O manager exposed over BLE.
 */

#pragma once

#include "digital/digital_pin_map.h"

#include <Arduino.h>

#include <vector>

namespace digital {

enum DigitalConfigFlags : uint8_t {
  kCfgPullUp = 1u << 0,
  kCfgPullDown = 1u << 1,
  kCfgNotifyOnChange = 1u << 2,
};

enum DigitalValueFlags : uint8_t {
  kValEnabled = 1u << 0,
};

struct DigitalConfigHeader {
  uint8_t numRecords;
  uint8_t flags;
  uint8_t reserved;
} __attribute__((packed));

struct DigitalConfigRecord {
  uint8_t channel;
  uint8_t mode;
  uint8_t flags;
  uint8_t reserved0;
  uint32_t pwmFrequencyHz;
} __attribute__((packed));

struct DigitalValueHeader {
  uint8_t numRecords;
  uint8_t flags;
  uint8_t reserved;
} __attribute__((packed));

struct DigitalValueRecord {
  uint8_t channel;
  uint8_t flags;
  uint16_t dutyPermille;
  uint8_t level;
  uint8_t reserved[3];
} __attribute__((packed));

struct PinState {
  DigitalMode mode;
  bool enabled;
  bool level;
  bool pullUp;
  bool pullDown;
  bool notifyOnChange;
  uint32_t pwmFrequencyHz;
  uint16_t dutyPermille;
};

static_assert(sizeof(DigitalConfigHeader) == 3, "DigitalConfigHeader must remain packed");
static_assert(sizeof(DigitalConfigRecord) == 8, "DigitalConfigRecord must remain packed");
static_assert(sizeof(DigitalValueHeader) == 3, "DigitalValueHeader must remain packed");
static_assert(sizeof(DigitalValueRecord) == 8, "DigitalValueRecord must remain packed");

bool init();
void resetToDefaults();

bool writeConfig(const uint8_t* data, uint16_t len);
std::vector<uint8_t> readConfig();

bool writeValue(const uint8_t* data, uint16_t len);
std::vector<uint8_t> readValue();

size_t getConfigPayloadSize();
size_t getValuePayloadSize();

} // namespace digital
