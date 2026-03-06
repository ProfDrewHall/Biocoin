# Documentation Index

- `BLE_PROTOCOL.md`: BLE characteristics, command framing, and payload notes.
- `BUILD_FLASH.md`: Build/flash notes and environment assumptions.
- `BATTERY_CALIBRATION.md`: BLE logging, curve fitting, and config update workflow for battery percent calibration.
- `CA_TECHNIQUE.md`: CA-specific parameter constraints, timing behavior, and ENDSEQ handling.
- `CV_TECHNIQUE.md`: CV parameter constraints, wakeup timing guards, and ENDSEQ behavior.
- `DPV_TECHNIQUE.md`: DPV parameter constraints, pulse timing guards, and ENDSEQ behavior.
- `SWV_TECHNIQUE.md`: SWV parameter constraints, timing guards, and ENDSEQ behavior.
- `OCP_TECHNIQUE.md`: OCP channel/rate constraints, wakeup guards, and ENDSEQ behavior.
- `TEMP_TECHNIQUE.md`: Temperature channel/rate constraints, wakeup guards, and ENDSEQ behavior.
- `IMP_TECHNIQUE.md`: Impedance parameter constraints, wakeup guards, and ISR behavior.
- `IONTOPHORESIS_TECHNIQUE.md`: Iontophoresis parameter and runtime safety behavior.
- `CONTRIBUTING.md`: Coding, naming, and documentation conventions for firmware changes.
- `POWER.md`: Low-power behavior and required SDK `WInterrupts` patch context.
- `RELEASE_CHECKLIST.md`: Pre-release checklist for firmware updates.

## Known Issues / Limitations

- Lowest-power operation still depends on SDK-level `WInterrupts` modifications (`SDK Modifications/WInterrupts-port.c` and `SDK Modifications/WInterrupts-port.h`).
- Build warning gating excludes vendor code warnings (for example, `src/drivers/ad5940.c`); project code warnings are expected to stay clean.
- BLE TX uses a bounded stream buffer; if producer throughput exceeds notify throughput, some bytes may be dropped (logged as warnings).
