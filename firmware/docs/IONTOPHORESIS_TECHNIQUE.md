# Iontophoresis Notes

## Host Parameter Constraints

- Payload layout (`IontophoresisParameterPayload`) is fixed-size packed data (`12` bytes).
- `samplingInterval` and `maxCurrent` must be finite and greater than `0`.
- `stimCurrent` must be finite, greater than or equal to `0`, and less than or equal to `maxCurrent`.

## Runtime Behavior

- Start has centralized failure cleanup (`cleanupStartFailure`) to guarantee SPI/peripheral shutdown.
- Stimulation safety is enforced in the monitoring task by comparing measured current against `maxCurrent`.
- On safety overcurrent, iontophoresis stops and publishes `CURRENT_LIMIT_EXCEEDED`.

## Timing Note

- This technique does not use AD5940 wakeup timer sequencing for sample cadence.
- Sampling cadence is controlled by the FreeRTOS monitoring task delay.
