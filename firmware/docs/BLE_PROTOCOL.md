# BLE Protocol Notes

## Characteristics

- Parameters: binary payload (`SensorType` + packed parameter struct bytes).
- Control: single-byte command (`START`, `STOP`).
- Status: single-byte `TestState`.
- Data: raw little-endian float stream.
- DigitalConfig: packed binary runtime digital-channel configuration table.
- DigitalValue: packed binary runtime digital-channel state/value table.

### Digital Characteristics

- DigitalConfig: UUID `0000152B-1212-EFDE-1523-785FEABC93AA`
  - Properties: `READ`, `WRITE`, `WRITE WITHOUT RESPONSE`
  - Purpose: persistent runtime channel configuration stored in RAM only
- DigitalValue: UUID `0000152C-1212-EFDE-1523-785FEABC93AA`
  - Properties: `READ`, `WRITE`, `WRITE WITHOUT RESPONSE`
  - Purpose: live channel state/readback and output control

Digital runtime behavior:

- Digital channel state persists across BLE disconnect/reconnect.
- Digital channel state does not persist across reboot.
- On boot, all exposed digital channels reset to floating/high-Z with PWM disabled.
- BLE addresses logical channels only; raw MCU pin numbers are not exposed over BLE.

Logical channels currently map to:

- `kAux0` -> `P0.11`
- `kAux1` -> `P0.12`
- `kAux2` -> `P0.24`
- `kAux3` -> `P0.25`

Digital modes:

- `kInput = 0`
- `kOutput = 1`
- `kFloating = 2`
- `kPWM = 3`

### DigitalConfig Payload

```c++
struct DigitalConfigHeader {
  uint8_t numRecords;
  uint8_t flags;
  uint8_t reserved;
} __attribute__((packed));

enum DigitalConfigFlags : uint8_t {
  kCfgPullUp         = 1 << 0,
  kCfgPullDown       = 1 << 1,
  kCfgNotifyOnChange = 1 << 2,
};

struct DigitalConfigRecord {
  uint8_t channel;         // DigitalChannel
  uint8_t mode;            // DigitalMode
  uint8_t flags;           // DigitalConfigFlags
  uint8_t reserved0;
  uint32_t pwmFrequencyHz; // valid only in PWM mode
} __attribute__((packed));
```

DigitalConfig behavior:

- Reads return the full config table for all exposed logical channels in fixed order.
- Writes may contain one or more records.
- Writes are partial updates; omitted channels remain unchanged.
- All records are validated first; if any record is invalid, the full write is rejected.
- Pull-up and pull-down are only valid in input mode.
- `pwmFrequencyHz` must be zero outside PWM mode.
- PWM frequency must be exactly representable by the firmware PWM backend or the write is rejected.

### DigitalValue Payload

```c++
struct DigitalValueHeader {
  uint8_t numRecords;
  uint8_t flags;
  uint8_t reserved;
} __attribute__((packed));

enum DigitalValueFlags : uint8_t {
  kValEnabled = 1 << 0,
};

struct DigitalValueRecord {
  uint8_t channel;      // DigitalChannel
  uint8_t flags;        // DigitalValueFlags
  uint16_t dutyPermille;// 0..1000, valid for PWM
  uint8_t level;        // 0 or 1
  uint8_t reserved[3];
} __attribute__((packed));
```

DigitalValue behavior:

- Reads return the full live state table for all exposed logical channels.
- Output mode:
  - `level` is the driven output level.
- Input mode:
  - `level` is sampled on read.
  - writes are rejected.
- Floating mode:
  - `level` is sampled on read.
  - writes are rejected.
- PWM mode:
  - `flags & kValEnabled` enables/disables PWM output.
  - `dutyPermille` sets PWM duty from `0..1000`.
  - `level` is ignored on write.

### Debug-only Characteristics (`DEBUG_MODE` builds only)

- Debug Battery Voltage: UUID `00001526-1212-EFDE-1523-785FEABC93AA`
  - Properties: `READ`, `NOTIFY`
  - Payload: 2-byte unsigned little-endian battery voltage in millivolts (`uint16`)
  - Example: payload `0xD2 0x0E` => `3794 mV`

## Payload Compatibility Rules

- Control writes must be exactly 1 byte.
- Parameters writes must be between 1 byte and the current BLE payload max (`MTU - ATT header`).
- Device name writes must be between 1 byte and `BLE_GAP_DEVNAME_MAX_LEN - 1`.
- DigitalConfig and DigitalValue writes must exactly match:
  - `sizeof(header) + numRecords * sizeof(record)`
- Digital payload pointer must not be null.
- Digital payload reserved header bytes/bits must be zero.
- Digital payload reserved record bytes/bits must be zero.
- Invalid payload length or null payload is ignored by firmware and logged as a warning.
- Host implementations should treat ignored writes as protocol errors and retry only with corrected payloads.

## TX Path Behavior

- Outgoing data is buffered in a bounded FreeRTOS stream buffer.
- Data is chunked to current negotiated MTU payload size.
- If producers outrun notify throughput, bytes can be dropped and a warning is logged.

## Host Guidance

- Always subscribe to Data before issuing `START`.
- Validate returned status transitions when changing techniques.
- Keep host-side struct packing and enum values in lock-step with firmware.
- Treat DigitalConfig and DigitalValue writes as strict binary protocols; the firmware rejects invalid enum values, reserved bits, and malformed lengths.
- Digital PWM frequency requests must use values exactly representable by the firmware PWM backend.
