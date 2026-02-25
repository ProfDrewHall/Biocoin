# Targeted Test Plan

This repository currently uses hardware-in-loop validation.  
Use this plan to cover parser/state-transition regressions with focused host-driven tests.

## Hardware Setup (EChem Loopback)

- For EChem unit-test runs, connect a `10 kOhm` resistor between `RE0` and `WE0`.
- Short `CE0` to `RE0`.

## Parser Tests

- Invalid payload length for each technique parameter struct.
- Null/empty payload handling for control and parameter writes.
- Unknown `SensorType` and unknown `SensorCmd` rejection.

## State Transition Tests

- `NOT_RUNNING -> RUNNING -> NOT_RUNNING` for each technique.
- Technique switch while stopped (`A -> B`) updates active sensor cleanly.
- Invalid parameter write leaves system in `INVALID_PARAMETERS` without crash.

## BLE TX Path Tests

- Data notifications after `START` with subscriber enabled.
- Buffer pressure scenario: verify dropped-byte warning appears when host is slow.
- Repeated start/stop cycles do not deadlock TX/data mover tasks.

## Suggested Execution

- Drive GATT writes/notifications from a host script and assert expected status bytes.
- Capture serial debug logs in `DEBUG_MODE` for task lifecycle and drop warnings.
