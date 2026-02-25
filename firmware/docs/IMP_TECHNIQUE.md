# Impedance (IMP/EIS) Notes

## Host Parameter Constraints

- Payload layout (`IMPParameterPayload`) is fixed-size packed data (`22` bytes).
- `samplingInterval`, `processingInterval`, `frequency`, and `Eac` must be finite and greater than `0`.
- `processingInterval` must be greater than or equal to `samplingInterval`.
- Boolean flag fields (`IMP_4wire`, `AC_coupled`) must be `0` or `1`.

## FIFO Threshold Behavior

- FIFO threshold is computed from interval ratio and rounded down to a multiple of `4` samples.
- Minimum threshold is `4`.

## Wakeup Timer Behavior

- Wakeup ticks are validated before start.
- Start fails if ticks are non-finite or too small for configured wake/sleep offsets.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- ISR drains FIFO for both `DATAFIFOTHRESH` and `ENDSEQ` events.
- On `ENDSEQ`, IMP stops and publishes `NOT_RUNNING` (or `ERROR` if stop fails).
