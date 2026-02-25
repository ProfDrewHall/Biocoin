# Square Wave Voltammetry (SWV) Notes

## Host Parameter Constraints

- Payload layout (`SWVParameterPayload`) is fixed-size packed data (`29` bytes).
- `processingInterval` and `pulsePeriod` must be finite and greater than `0`.
- `Estep` must be non-zero.
- `channel` must be in range `[0, 3]`.
- Voltage points (`Estart`, `Estop`, `Eamplitude`) must remain inside DAC headroom derived from `VzeroStart`.

## FIFO Threshold Behavior

- FIFO threshold is computed as `round(processingInterval * 1000 / (pulsePeriod / 2))`.
- Threshold is clamped to `[1, 1023]`.

## Wakeup Timer Behavior

- Sample-delay and pulse-width timing ticks are validated before start.
- Start fails if computed ticks are non-finite or too small after timing offsets.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- ISR drains FIFO for both `DATAFIFOTHRESH` and `ENDSEQ` events.
- On `ENDSEQ`, SWV stops and publishes `NOT_RUNNING` (or `ERROR` if stop fails).
