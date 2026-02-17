# Revision History

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
