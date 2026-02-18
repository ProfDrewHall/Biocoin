# Documentation Index

- `BLE_PROTOCOL.md`: BLE characteristics, command framing, and payload notes.
- `BUILD_FLASH.md`: Build/flash notes and environment assumptions.
- `POWER.md`: Low-power behavior and required SDK `WInterrupts` patch context.
- `RELEASE_CHECKLIST.md`: Pre-release checklist for firmware updates.

## Known Issues / Limitations

- Lowest-power operation still depends on SDK-level `WInterrupts` modifications (`SDK Modifications/WInterrupts-port.c` and `SDK Modifications/WInterrupts-port.h`).
- Build warning gating excludes vendor code warnings (for example, `src/drivers/ad5940.c`); project code warnings are expected to stay clean.
- BLE TX uses a bounded stream buffer; if producer throughput exceeds notify throughput, some bytes may be dropped (logged as warnings).
