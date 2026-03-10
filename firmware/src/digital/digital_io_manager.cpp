/**
 * @file digital_io_manager.cpp
 * @brief Persistent runtime digital I/O manager implementation.
 */

#include "digital/digital_io_manager.h"

#include "power/power.h"
#include "util/debug_log.h"

#include <HardwarePWM.h>
#include <nrf_gpio.h>

#include <array>
#include <cstring>

namespace digital {
namespace {

constexpr uint32_t kDigitalPWMOwnerToken = 0x4F494744u; // 'DGIO'
constexpr uint8_t kDigitalPWMModuleIndex =
#ifdef NRF_PWM3
    3u;
#else
    2u;
#endif
constexpr uint8_t kPWMFlagMask = kValEnabled;
constexpr uint8_t kConfigFlagMask = kCfgPullUp | kCfgPullDown | kCfgNotifyOnChange;

struct PWMConfig {
  uint8_t clockDiv;
  uint16_t countertop;
};

std::array<PinState, kNumDigitalPins> gPinStates = {};
bool gInitialized = false;
bool gPWMAvailable = false;

HardwarePWM& pwmModule() {
  return *HwPWMx[kDigitalPWMModuleIndex];
}

const DigitalPinInfo* findPinInfo(DigitalChannel channel, size_t* indexOut = nullptr) {
  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    if (kDigitalPins[i].channel == channel) {
      if (indexOut != nullptr) *indexOut = i;
      return &kDigitalPins[i];
    }
  }
  return nullptr;
}

bool isKnownChannelValue(uint8_t channelValue, DigitalChannel* channelOut = nullptr) {
  const DigitalChannel channel = static_cast<DigitalChannel>(channelValue);
  size_t unusedIndex = 0;
  if (findPinInfo(channel, &unusedIndex) == nullptr) return false;
  if (channelOut != nullptr) *channelOut = channel;
  return true;
}

bool isKnownModeValue(uint8_t modeValue, DigitalMode* modeOut = nullptr) {
  switch (static_cast<DigitalMode>(modeValue)) {
  case DigitalMode::kInput:
  case DigitalMode::kOutput:
  case DigitalMode::kFloating:
  case DigitalMode::kPWM:
    if (modeOut != nullptr) *modeOut = static_cast<DigitalMode>(modeValue);
    return true;
  default:
    return false;
  }
}

uint8_t pinToNrfPin(uint32_t pin) {
  return g_ADigitalPinMap[pin];
}

bool readRawPin(uint32_t pin) {
  uint32_t nrfPin = pinToNrfPin(pin);
  NRF_GPIO_Type* port = nrf_gpio_pin_port_decode(&nrfPin);
  return ((port->IN >> nrfPin) & 0x1u) != 0u;
}

void releasePWMPin(const DigitalPinInfo& info) {
  if (!gPWMAvailable) return;
  HardwarePWM& pwm = pwmModule();
  if (pwm.checkPin(info.gpioPin)) {
    pwm.removePin(info.gpioPin);
    dbgInfo(String("Digital PWM detached from ") + info.name);
  }

  if (pwm.usedChannelCount() == 0u && pwm.enabled()) {
    pwm.stop();
  }
}

bool computePWMConfig(uint32_t frequencyHz, PWMConfig* out) {
  if (out == nullptr || frequencyHz == 0u) return false;

  constexpr uint32_t kBaseClockHz = 16000000u;
  constexpr uint8_t kClockDivisors[] = {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u};
  constexpr uint8_t kClockDivRegisters[] = {
      PWM_PRESCALER_PRESCALER_DIV_1, PWM_PRESCALER_PRESCALER_DIV_2, PWM_PRESCALER_PRESCALER_DIV_4,
      PWM_PRESCALER_PRESCALER_DIV_8, PWM_PRESCALER_PRESCALER_DIV_16, PWM_PRESCALER_PRESCALER_DIV_32,
      PWM_PRESCALER_PRESCALER_DIV_64, PWM_PRESCALER_PRESCALER_DIV_128};

  for (size_t i = 0; i < (sizeof(kClockDivisors) / sizeof(kClockDivisors[0])); ++i) {
    const uint32_t divisor = kClockDivisors[i];
    const uint32_t denom = divisor * frequencyHz;
    if (denom == 0u) continue;
    if ((kBaseClockHz % denom) != 0u) continue;

    const uint32_t countertop = kBaseClockHz / denom;
    if (countertop == 0u || countertop > 32767u) continue;

    out->clockDiv = kClockDivRegisters[i];
    out->countertop = static_cast<uint16_t>(countertop);
    return true;
  }

  return false;
}

uint16_t dutyPermilleToCountertop(uint16_t dutyPermille, uint16_t countertop) {
  if (dutyPermille >= 1000u) return countertop;
  const uint32_t scaled = static_cast<uint32_t>(dutyPermille) * static_cast<uint32_t>(countertop);
  return static_cast<uint16_t>((scaled + 500u) / 1000u);
}

bool applyPWMOutput(const DigitalPinInfo& info, const PinState& state) {
  if (!gPWMAvailable) {
    dbgError(String("PWM unavailable for ") + info.name);
    return false;
  }

  PWMConfig pwmConfig = {};
  if (!computePWMConfig(state.pwmFrequencyHz, &pwmConfig)) {
    dbgError(String("Unsupported PWM frequency for ") + info.name + ": " + state.pwmFrequencyHz + " Hz");
    return false;
  }

  if (!state.enabled) {
    releasePWMPin(info);
    power::disconnectInputGPIO(info.gpioPin);
    dbgInfo(String("Digital PWM disabled on ") + info.name);
    return true;
  }

  HardwarePWM& pwm = pwmModule();
  pwm.setClockDiv(pwmConfig.clockDiv);
  pwm.setMaxValue(pwmConfig.countertop);
  if (!pwm.checkPin(info.gpioPin) && !pwm.addPin(info.gpioPin)) {
    dbgError(String("Failed to attach PWM pin for ") + info.name);
    return false;
  }

  const uint16_t compareValue = dutyPermilleToCountertop(state.dutyPermille, pwmConfig.countertop);
  if (!pwm.writePin(info.gpioPin, compareValue)) {
    dbgError(String("Failed to update PWM output for ") + info.name);
    return false;
  }

  dbgInfo(String("Digital PWM configured for ") + info.name + ": freq=" + state.pwmFrequencyHz + "Hz duty=" +
          state.dutyPermille + " enabled=" + state.enabled);
  return true;
}

void applyFloating(const DigitalPinInfo& info) {
  releasePWMPin(info);
  power::disconnectInputGPIO(info.gpioPin);
}

void applyInput(const DigitalPinInfo& info, const PinState& state) {
  releasePWMPin(info);
  const power::PullConfig pull = state.pullUp   ? power::PullConfig::Pullup
                                : state.pullDown ? power::PullConfig::Pulldown
                                                 : power::PullConfig::Disabled;
  power::reconnectInputGPIO(info.gpioPin, pull);
}

void applyOutput(const DigitalPinInfo& info, const PinState& state) {
  releasePWMPin(info);
  pinMode(info.gpioPin, OUTPUT);
  digitalWrite(info.gpioPin, state.level ? HIGH : LOW);
}

bool applyStateToHardware(const DigitalPinInfo& info, const PinState& state) {
  switch (state.mode) {
  case DigitalMode::kFloating:
    applyFloating(info);
    dbgInfo(String("Digital mode -> FLOATING on ") + info.name);
    return true;
  case DigitalMode::kInput:
    applyInput(info, state);
    dbgInfo(String("Digital mode -> INPUT on ") + info.name);
    return true;
  case DigitalMode::kOutput:
    applyOutput(info, state);
    dbgInfo(String("Digital mode -> OUTPUT on ") + info.name + " level=" + state.level);
    return true;
  case DigitalMode::kPWM:
    dbgInfo(String("Digital mode -> PWM on ") + info.name);
    return applyPWMOutput(info, state);
  default:
    return false;
  }
}

bool validateConfigRecord(const DigitalConfigRecord& record, String* reason) {
  DigitalChannel channel = DigitalChannel::kAux0;
  if (!isKnownChannelValue(record.channel, &channel)) {
    *reason = String("Invalid digital channel: ") + record.channel;
    return false;
  }

  DigitalMode mode = DigitalMode::kFloating;
  if (!isKnownModeValue(record.mode, &mode)) {
    *reason = String("Invalid digital mode for channel ") + record.channel;
    return false;
  }

  if (record.reserved0 != 0u) {
    *reason = String("Digital config reserved byte must be zero for channel ") + record.channel;
    return false;
  }

  if ((record.flags & ~kConfigFlagMask) != 0u) {
    *reason = String("Digital config reserved flag bits set for channel ") + record.channel;
    return false;
  }

  const DigitalPinInfo* info = findPinInfo(channel);
  if (info == nullptr) {
    *reason = String("Digital channel not exposed: ") + record.channel;
    return false;
  }

  const bool pullUpEnabled = (record.flags & kCfgPullUp) != 0u;
  const bool pullDownEnabled = (record.flags & kCfgPullDown) != 0u;
  if (pullUpEnabled && pullDownEnabled) {
    *reason = String("Pull-up and pull-down cannot both be enabled for channel ") + record.channel;
    return false;
  }

  if ((pullUpEnabled || pullDownEnabled) && mode != DigitalMode::kInput) {
    *reason = String("Pull configuration is only valid in input mode for channel ") + record.channel;
    return false;
  }

  switch (mode) {
  case DigitalMode::kInput:
    if (record.pwmFrequencyHz != 0u) {
      *reason = String("PWM frequency must be zero outside PWM mode for channel ") + record.channel;
      return false;
    }
    return true;
  case DigitalMode::kOutput:
    if (record.pwmFrequencyHz != 0u) {
      *reason = String("PWM frequency must be zero outside PWM mode for channel ") + record.channel;
      return false;
    }
    return true;
  case DigitalMode::kFloating:
    if (record.pwmFrequencyHz != 0u) {
      *reason = String("PWM frequency must be zero outside PWM mode for channel ") + record.channel;
      return false;
    }
    return true;
  case DigitalMode::kPWM: {
    if (record.pwmFrequencyHz == 0u) {
      *reason = String("PWM frequency must be > 0 for channel ") + record.channel;
      return false;
    }
    PWMConfig pwmConfig = {};
    if (!computePWMConfig(record.pwmFrequencyHz, &pwmConfig)) {
      *reason = String("PWM frequency is not representable for channel ") + record.channel;
      return false;
    }
    return true;
  }
  default:
    *reason = "Unknown digital mode";
    return false;
  }
}

bool validateValueRecord(const DigitalValueRecord& record, const PinState& currentState, String* reason) {
  DigitalChannel channel = DigitalChannel::kAux0;
  if (!isKnownChannelValue(record.channel, &channel)) {
    *reason = String("Invalid digital channel: ") + record.channel;
    return false;
  }

  if ((record.flags & ~kPWMFlagMask) != 0u) {
    *reason = String("Digital value reserved flag bits set for channel ") + record.channel;
    return false;
  }

  if (record.reserved[0] != 0u || record.reserved[1] != 0u || record.reserved[2] != 0u) {
    *reason = String("Digital value reserved bytes must be zero for channel ") + record.channel;
    return false;
  }

  switch (currentState.mode) {
  case DigitalMode::kInput:
    *reason = String("Digital value writes are rejected in input mode for channel ") + record.channel;
    return false;
  case DigitalMode::kFloating:
    *reason = String("Digital value writes are rejected in floating mode for channel ") + record.channel;
    return false;
  case DigitalMode::kOutput:
    if (record.flags != 0u) {
      *reason = String("Output-mode value flags must be zero for channel ") + record.channel;
      return false;
    }
    if (record.dutyPermille != 0u) {
      *reason = String("Output-mode duty must be zero for channel ") + record.channel;
      return false;
    }
    if (record.level > 1u) {
      *reason = String("Output-mode level must be 0 or 1 for channel ") + record.channel;
      return false;
    }
    return true;
  case DigitalMode::kPWM:
    if (record.dutyPermille > 1000u) {
      *reason = String("PWM duty must be within 0..1000 for channel ") + record.channel;
      return false;
    }
    return true;
  default:
    *reason = "Unknown channel mode";
    return false;
  }
}

std::vector<uint8_t> serializeConfigPayload() {
  std::vector<uint8_t> payload(sizeof(DigitalConfigHeader) + (kNumDigitalPins * sizeof(DigitalConfigRecord)));
  DigitalConfigHeader header = {static_cast<uint8_t>(kNumDigitalPins), 0u, 0u};
  memcpy(payload.data(), &header, sizeof(header));

  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    const PinState& state = gPinStates[i];
    DigitalConfigRecord record = {};
    record.channel = static_cast<uint8_t>(kDigitalPins[i].channel);
    record.mode = static_cast<uint8_t>(state.mode);
    if (state.pullUp) record.flags |= kCfgPullUp;
    if (state.pullDown) record.flags |= kCfgPullDown;
    if (state.notifyOnChange) record.flags |= kCfgNotifyOnChange;
    record.pwmFrequencyHz = (state.mode == DigitalMode::kPWM) ? state.pwmFrequencyHz : 0u;
    memcpy(payload.data() + sizeof(DigitalConfigHeader) + (i * sizeof(DigitalConfigRecord)), &record, sizeof(record));
  }

  return payload;
}

std::vector<uint8_t> serializeValuePayload() {
  std::vector<uint8_t> payload(sizeof(DigitalValueHeader) + (kNumDigitalPins * sizeof(DigitalValueRecord)));
  DigitalValueHeader header = {static_cast<uint8_t>(kNumDigitalPins), 0u, 0u};
  memcpy(payload.data(), &header, sizeof(header));

  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    PinState liveState = gPinStates[i];
    DigitalValueRecord record = {};
    record.channel = static_cast<uint8_t>(kDigitalPins[i].channel);

    if (liveState.mode == DigitalMode::kPWM) {
      if (liveState.enabled) record.flags |= kValEnabled;
    } else {
      liveState.enabled = false;
    }

    if (liveState.mode == DigitalMode::kInput || liveState.mode == DigitalMode::kOutput ||
        liveState.mode == DigitalMode::kFloating) {
      liveState.level = readRawPin(kDigitalPins[i].gpioPin);
    }

    record.dutyPermille = (liveState.mode == DigitalMode::kPWM) ? liveState.dutyPermille : 0u;
    record.level = liveState.level ? 1u : 0u;
    memcpy(payload.data() + sizeof(DigitalValueHeader) + (i * sizeof(DigitalValueRecord)), &record, sizeof(record));
  }

  return payload;
}

} // namespace

bool init() {
  if (!gInitialized) {
    HardwarePWM& pwm = pwmModule();
    gPWMAvailable = pwm.takeOwnership(kDigitalPWMOwnerToken);
    if (!gPWMAvailable) {
      dbgError("Digital I/O failed to reserve dedicated PWM peripheral");
    }
    gInitialized = true;
  }

  resetToDefaults();
  return true;
}

void resetToDefaults() {
  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    PinState state = {};
    state.mode = kDigitalPins[i].defaultMode;
    state.enabled = false;
    state.level = false;
    state.pullUp = false;
    state.pullDown = false;
    state.notifyOnChange = false;
    state.pwmFrequencyHz = 0u;
    state.dutyPermille = 0u;
    gPinStates[i] = state;
    applyFloating(kDigitalPins[i]);
  }

  dbgInfo("Digital I/O reset to defaults");
}

bool writeConfig(const uint8_t* data, uint16_t len) {
  dbgInfo("Received digital config write");

  if (data == nullptr) {
    dbgError("Rejected digital config write: null payload");
    return false;
  }
  if (len < sizeof(DigitalConfigHeader)) {
    dbgError(String("Rejected digital config write: short payload len=") + len);
    return false;
  }

  DigitalConfigHeader header = {};
  memcpy(&header, data, sizeof(header));
  const size_t expectedLen =
      sizeof(DigitalConfigHeader) + (static_cast<size_t>(header.numRecords) * sizeof(DigitalConfigRecord));
  if (len != expectedLen) {
    dbgError(String("Rejected digital config write: len=") + len + " expected=" + expectedLen);
    return false;
  }
  if (header.flags != 0u || header.reserved != 0u) {
    dbgError("Rejected digital config write: invalid header");
    return false;
  }

  std::array<PinState, kNumDigitalPins> nextStates = gPinStates;
  std::array<bool, kNumDigitalPins> touched = {};

  for (uint8_t i = 0; i < header.numRecords; ++i) {
    DigitalConfigRecord record = {};
    memcpy(&record, data + sizeof(DigitalConfigHeader) + (static_cast<size_t>(i) * sizeof(DigitalConfigRecord)),
           sizeof(record));

    String reason;
    if (!validateConfigRecord(record, &reason)) {
      dbgError(String("Rejected digital config write: ") + reason);
      return false;
    }

    size_t pinIndex = 0;
    const DigitalPinInfo* info = findPinInfo(static_cast<DigitalChannel>(record.channel), &pinIndex);
    if (info == nullptr) {
      dbgError(String("Rejected digital config write: unknown channel ") + record.channel);
      return false;
    }
    if (touched[pinIndex]) {
      dbgError(String("Rejected digital config write: duplicate channel ") + record.channel);
      return false;
    }
    touched[pinIndex] = true;

    PinState& state = nextStates[pinIndex];
    state.mode = static_cast<DigitalMode>(record.mode);
    state.pullUp = (record.flags & kCfgPullUp) != 0u;
    state.pullDown = (record.flags & kCfgPullDown) != 0u;
    state.notifyOnChange = (record.flags & kCfgNotifyOnChange) != 0u;
    state.pwmFrequencyHz = (state.mode == DigitalMode::kPWM) ? record.pwmFrequencyHz : 0u;
    if (state.mode != DigitalMode::kPWM) {
      state.enabled = false;
      state.dutyPermille = 0u;
    }
  }

  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    if (touched[i] && !applyStateToHardware(kDigitalPins[i], nextStates[i])) {
      dbgError(String("Rejected digital config write during apply for ") + kDigitalPins[i].name);
      return false;
    }
  }

  gPinStates = nextStates;
  return true;
}

std::vector<uint8_t> readConfig() {
  return serializeConfigPayload();
}

bool writeValue(const uint8_t* data, uint16_t len) {
  dbgInfo("Received digital value write");

  if (data == nullptr) {
    dbgError("Rejected digital value write: null payload");
    return false;
  }
  if (len < sizeof(DigitalValueHeader)) {
    dbgError(String("Rejected digital value write: short payload len=") + len);
    return false;
  }

  DigitalValueHeader header = {};
  memcpy(&header, data, sizeof(header));
  const size_t expectedLen =
      sizeof(DigitalValueHeader) + (static_cast<size_t>(header.numRecords) * sizeof(DigitalValueRecord));
  if (len != expectedLen) {
    dbgError(String("Rejected digital value write: len=") + len + " expected=" + expectedLen);
    return false;
  }
  if (header.flags != 0u || header.reserved != 0u) {
    dbgError("Rejected digital value write: invalid header");
    return false;
  }

  std::array<PinState, kNumDigitalPins> nextStates = gPinStates;
  std::array<bool, kNumDigitalPins> touched = {};

  for (uint8_t i = 0; i < header.numRecords; ++i) {
    DigitalValueRecord record = {};
    memcpy(&record, data + sizeof(DigitalValueHeader) + (static_cast<size_t>(i) * sizeof(DigitalValueRecord)),
           sizeof(record));

    size_t pinIndex = 0;
    const DigitalPinInfo* info = findPinInfo(static_cast<DigitalChannel>(record.channel), &pinIndex);
    if (info == nullptr) {
      dbgError(String("Rejected digital value write: invalid channel ") + record.channel);
      return false;
    }
    if (touched[pinIndex]) {
      dbgError(String("Rejected digital value write: duplicate channel ") + record.channel);
      return false;
    }

    String reason;
    if (!validateValueRecord(record, nextStates[pinIndex], &reason)) {
      dbgError(String("Rejected digital value write: ") + reason);
      return false;
    }
    touched[pinIndex] = true;

    PinState& state = nextStates[pinIndex];
    if (state.mode == DigitalMode::kOutput) {
      state.level = (record.level != 0u);
    } else if (state.mode == DigitalMode::kPWM) {
      state.enabled = (record.flags & kValEnabled) != 0u;
      state.dutyPermille = record.dutyPermille;
    }
  }

  for (size_t i = 0; i < kNumDigitalPins; ++i) {
    if (touched[i] && !applyStateToHardware(kDigitalPins[i], nextStates[i])) {
      dbgError(String("Rejected digital value write during apply for ") + kDigitalPins[i].name);
      return false;
    }
  }

  gPinStates = nextStates;
  return true;
}

std::vector<uint8_t> readValue() {
  return serializeValuePayload();
}

size_t getConfigPayloadSize() {
  return sizeof(DigitalConfigHeader) + (kNumDigitalPins * sizeof(DigitalConfigRecord));
}

size_t getValuePayloadSize() {
  return sizeof(DigitalValueHeader) + (kNumDigitalPins * sizeof(DigitalValueRecord));
}

} // namespace digital
