# Revision History

## Unreleased (since last push to `origin/main`) - 2026-02-18

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
