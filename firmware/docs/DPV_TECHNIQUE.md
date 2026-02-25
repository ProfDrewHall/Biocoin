# Differential Pulse Voltammetry (DPV) Notes

## Host Parameter Constraints

- Payload layout (`DPVParameterPayload`) is fixed-size packed data (`33` bytes).
- `processingInterval`, `pulseWidth`, and `pulsePeriod` must be finite and greater than `0`.
- `pulsePeriod` must be greater than or equal to `pulseWidth`.
- `Estep` must be non-zero.
- `channel` must be in range `[0, 3]`.
- Voltage points (`Estart`, `Estop`, `Epulse`) must remain inside DAC headroom derived from `VzeroStart`.

## FIFO Threshold Behavior

- FIFO threshold is computed as `round(processingInterval * 1000 / pulseWidth)`.
- Threshold is clamped to `[1, 1023]`.

## Wakeup Timer Behavior

- High/low pulse timing ticks are validated before start.
- Start fails if any computed wakeup ticks are non-finite or too small after timing offsets.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- ISR drains FIFO for both `DATAFIFOTHRESH` and `ENDSEQ` events.
- On `ENDSEQ`, DPV stops and publishes `NOT_RUNNING` (or `ERROR` if stop fails).
