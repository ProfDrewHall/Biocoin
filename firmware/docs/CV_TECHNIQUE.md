# Cyclic Voltammetry (CV) Notes

## Host Parameter Constraints

- Payload layout (`CVParameterPayload`) is fixed-size packed data (`29` bytes).
- `processingInterval` and `pulseWidth` must be finite and greater than `0`.
- `Estep` must be non-zero.
- `channel` must be in range `[0, 3]`.
- Voltage points (`Estart`, `Evertex1`, `Evertex2`) must remain inside DAC headroom derived from `VzeroStart`.

## FIFO Threshold Behavior

- FIFO threshold is computed as `round(processingInterval * 1000 / pulseWidth)`.
- Threshold is clamped to `[1, 1023]`.

## Wakeup Timer Behavior

- Wakeup timer settings are validated before start.
- Start fails if computed wake/sleep ticks are non-finite or too small after timing offsets.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- ISR drains FIFO for both `DATAFIFOTHRESH` and `ENDSEQ` events.
- On `ENDSEQ`, CV stops and publishes `NOT_RUNNING` (or `ERROR` if stop fails).
