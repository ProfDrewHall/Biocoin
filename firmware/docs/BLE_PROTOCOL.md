# BLE Protocol Notes

## Characteristics

- Parameters: binary payload (`SensorType` + packed parameter struct bytes).
- Control: single-byte command (`START`, `STOP`).
- Status: single-byte `TestState`.
- Data: raw little-endian float stream.

## Payload Compatibility Rules

- Control writes must be exactly 1 byte.
- Parameters writes must be between 1 byte and the current BLE payload max (`MTU - ATT header`).
- Device name writes must be between 1 byte and `BLE_GAP_DEVNAME_MAX_LEN - 1`.
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
