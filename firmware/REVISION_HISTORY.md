# Revision History

## v1.1.3 - 2026-03-06
- Added battery coefficients for FP10AAB36 battery
- Task lifecycle alignment:
  - Added full heartbeat LED task lifecycle control (`startHeartbeatTask` + `stopHeartbeatTask`) with cooperative notify-based shutdown in `src/power/led_task.cpp`.
- Battery API naming alignment:
  - Renamed battery APIs:
    - `readVoltage(...)` -> `readBatteryVoltageV(...)`
    - `readLevel(...)` -> `readBatteryPercent(...)`
    - `mapBatteryLevel(...)` -> `voltageToPercent(...)`
- BLE protocol/debug documentation:
  - Added debug-only battery-voltage characteristic details to `docs/BLE_PROTOCOL.md` (UUID `0x1526`, `uint16` little-endian mV payload).
- Repository alignment cleanup:
  - Standardized product naming to `Biocoin` across firmware/docs defaults/comments.
  - Fixed stale variant comment in `platformio.ini`.
  - Updated pre-push/release flow alignment so `.githooks/pre-push` runs sanity + smoke checks before `debug`/`release` builds and artifact sync.
  - Refreshed `scripts/smoke_sensor_modes.sh` paths for current `src/sensors/core` + `src/sensors/techniques/*` layout.
  - Expanded `.gitignore` hygiene for Python/test artifacts and local outputs (`__pycache__`, logs, CSV/PNG, temp, coverage) in both firmware scripts and `software/`.
- Battery calibration and model update:
  - Added BLE battery logging + analysis workflow docs and scripts (`docs/BATTERY_CALIBRATION.md`, `scripts/log_battery_ble.py`, `scripts/analyze_battery_curve.py`).
  - Updated battery sigmoid mapping constants in `src/HWConfig/config.h` from fitted calibration outputs.
- Debug BLE battery/AFE controls (debug builds only):
  - Added debug battery-voltage characteristic support and host refresh command handling.
  - Added debug AFE burn-load control characteristic (`0x1527`) for controlled discharge testing.
  - Added disconnect-time cleanup to ensure debug AFE burn mode is turned off.
- BLE logger robustness improvements:
  - Added full discovered-device/UUID scan printing mode (`--print-discovered`).
  - Added Windows-focused connection retry + uncached service discovery fallback handling.
  - Added graceful recovery/exit behavior for mid-session `Not connected` disconnects.
- Debug startup reliability:
  - Capped debug serial wait at startup so BLE advertising still starts without an attached serial monitor.

## v1.1.2 - 2026-02-25

- Technique refactor for reuse and clarity:
  - Added shared setup pipeline helpers in `src/sensors/techniques/setup_measurement_helpers.h`.
  - Added shared wakeup-tick math helpers in `src/sensors/techniques/technique_lifecycle_helpers.h`.
  - Added shared runtime interrupt/FIFO helpers in `src/sensors/techniques/technique_runtime_helpers.h`.
  - Added shared technique payload parse/log helpers in `src/sensors/techniques/parameter_io_helpers.h`.
- Sensor-runtime reliability fixes:
  - Restored post-RTIA-calibration re-arm flow (FIFO/interrupt/sequencer generator) in technique setup paths to avoid no-data runs.
  - Added a CA stop-path best-effort final FIFO drain to reduce dropped tail samples.
  - Updated data mover wake path to ISR-safe FreeRTOS APIs and synchronized stop signaling with an atomic flag.
- Validation and API cleanup:
  - Consolidated exact-struct payload parsing through `payload_validation` to remove duplicate parse logic.
  - Removed unused `processData()` declarations from technique headers since it is not part of the `Sensor` base interface.
- Documentation/test note updates:
  - Added unit-test hardware hookup note (10k resistor RE0-WE0, CE0 shorted to RE0) to test docs.
  - Added tracked pre-push hook at `.githooks/pre-push` to build `debug` and `release` and copy artifacts into `build/<env>/`.

## v1.1.1 - 2026-02-18

- Refactored BLE transmit pipeline:
  - Replaced misspelled `tansmitdata_task.cpp` with `transmitdata_task.cpp`.
  - Moved TX path to bounded stream-buffer flow with clearer task lifecycle handling.
- Hardened BLE/GATT payload validation:
  - Added centralized helper `src/util/payload_validation.h`.
  - Added/expanded length checks for control, parameters, and device-name writes.
  - Removed stale/dead GATT declaration from `src/bluetooth/gatt.h`.
- Sensor manager and task cleanup:
  - Improved active-sensor lifecycle handling and status update paths.
  - Updated sensor/data mover control flow for cleaner start/stop behavior.
- Source-level cleanup and consistency:
  - Removed redundant includes and `using namespace` remnants.
  - Normalized sensor/driver filenames to consistent lowercase snake_case and updated includes accordingly.
  - Applied line-ending normalization and formatting consistency updates across source files.
  - Fixed compile issues/warnings introduced during cleanup (missing include and signed/unsigned compare in impedance path).
- CI and automation additions:
  - Added `scripts/sanity_checks.sh` for include/path/style/line-ending checks.
  - Added `scripts/smoke_sensor_modes.sh` for sensor-mode coverage smoke checks.
  - Added `scripts/check_project_warnings.sh` to gate non-vendor compiler warnings.
  - Added GitHub Actions workflow `../.github/workflows/firmware-sanity.yml` to run checks in CI.
- Documentation expansion:
  - Added docs set: `docs/README.md`, `docs/BLE_PROTOCOL.md`, `docs/BUILD_FLASH.md`, `docs/POWER.md`,
    and `docs/RELEASE_CHECKLIST.md`.
  - Updated top-level `README.md` for current architecture, BLE TX behavior, and SDK low-power notes.
  - Added targeted host/HIL test plan in `test/README.md`.

## v1.1.0 (2026-02-17)

- Added **SWV (Square Wave Voltammetry)** support in firmware and documented SWV payloads.
- Refactored hardware configuration:
  - Split `src/HWConfig/constants.h` into `src/HWConfig/build_checks.h`, `src/HWConfig/config.h`, and `src/HWConfig/pins.h`.
  - Kept `src/HWConfig/constants.h` as the aggregator include.
- Migrated to repo-local variant configuration:
  - Added `src/variants/biocoin/variant.h` and `src/variants/biocoin/variant.cpp`.
  - Added local variant marker macro `BIOCOIN_LOCAL_VARIANT`.
  - Updated `platformio.ini` to use local variant (`board_build.variants_dir`, `board_build.variant`).
- Added compile-time environment checks for variant selection and board assumptions (LFRC/LFXO, QSPI, pin count).
- Hardened battery/BLE behavior:
  - Added cooperative task stop helper `src/util/task_sync.h`.
  - Updated `src/battery/battery_task.cpp` to use notify-based stop/wait and safer restart behavior.
  - Switched battery service updates from `blebas.write(...)` to `blebas.notify(...)`.
  - Added `bledis.setFirmwareRev(kFirmwareRev)` in `src/bluetooth/bluetooth.cpp`.
- Improved storage device-name handling:
  - Replaced hardcoded max name length with `BLE_GAP_DEVNAME_MAX_LEN + 1`.
  - Added defensive name clamping in `storage::writeDeviceName(...)`.
- Code quality cleanup:
  - Removed file-scope `using namespace ...` across `src/*.cpp` and applied explicit qualification.
  - Added module/API comments to updated headers and source files.
- Documentation updates:
  - Updated `README.md` for current project structure and SDK notes.
  - Added note that SDK `WInterrupts` modifications are still needed for lowest-power operation.
