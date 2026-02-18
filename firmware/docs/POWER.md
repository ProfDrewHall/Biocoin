# Power Notes

## Runtime Strategy

- Peripheral blocks are enabled only when needed for active measurements.
- Multiple input GPIOs are explicitly disconnected to minimize leakage.
- BLE TX uses bounded buffering to avoid unbounded memory growth.

## SDK Patch Requirement

- Lowest-power operation still requires SDK-level `WInterrupts` modifications.
- See:
  - `SDK Modifications/WInterrupts-port.c`
  - `SDK Modifications/WInterrupts-port.h`

## Validation Suggestions

- Measure current in idle advertising, connected idle, and active measurement modes.
- Re-check power after any pin-map or interrupt-path change.
