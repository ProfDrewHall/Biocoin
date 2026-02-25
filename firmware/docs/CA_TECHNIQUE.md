# Chronoamperometry (CA) Notes

This note captures CA-specific runtime constraints and behavior so the same rules can be applied to other electrochemistry techniques.

## Host Parameter Constraints

- Payload layout (`CAParameterPayload`) is fixed-size packed data (`17` bytes).
- `samplingInterval` and `processingInterval` must be finite and greater than `0`.
- `processingInterval` must be greater than or equal to `samplingInterval`.
- `channel` must be in range `[0, 3]`.
- `pulsePotential` must remain within bias limits derived from current `Vzero`:
  - `minBias = AD5940_MIN_DAC_OUTPUT - Vzero`
  - `maxBias = AD5940_MAX_DAC_OUTPUT - Vzero`

## FIFO Threshold Behavior

- FIFO threshold is computed as:
  - `round(processingInterval / samplingInterval)`
- Threshold is clamped to:
  - minimum: `kCAMinFifoThreshold`
  - maximum: `kCAMaxFifoThreshold`

## Wakeup Timer Behavior

- CA builds wakeup timer config from measured `LFOSCFreq` and configured sampling interval.
- Wake period uses:
  - `kCAWuptSleepTicks`
  - `kCAWuptWakeSubtractTicks`
- Start fails with cleanup if computed wake ticks are non-finite or too small.

## ENDSEQ Handling

- ISR drains FIFO on both `DATAFIFOTHRESH` and `ENDSEQ`.
- On `ENDSEQ`, CA stops the technique and publishes:
  - `NOT_RUNNING` on successful stop
  - `ERROR` if stop fails
