# Temperature Technique Notes

## Host Parameter Constraints

- Payload layout (`TempParameterPayload`) is fixed-size packed data (`9` bytes).
- `samplingInterval` and `processingInterval` must be finite and greater than `0`.
- `processingInterval` must be greater than or equal to `samplingInterval`.
- `channel` must be one of the allowed temperature mux channels.

## FIFO Threshold Behavior

- FIFO threshold is computed as `round(processingInterval / samplingInterval)`.
- Threshold is clamped to `[1, 1023]`.

## Wakeup Timer Behavior

- Wakeup ticks are validated before start.
- Start fails if ticks are non-finite or too small for configured wake/sleep offsets.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- ISR drains FIFO for both `DATAFIFOTHRESH` and `ENDSEQ` events.
- On `ENDSEQ`, TEMP stops and publishes `NOT_RUNNING` (or `ERROR` if stop fails).
