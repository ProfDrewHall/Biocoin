# Release Checklist

## Minimum Acceptance (must pass)

- [ ] `pio run` completes for release target(s).
- [ ] `bash scripts/sanity_checks.sh` passes.
- [ ] `bash scripts/smoke_sensor_modes.sh` passes.
- [ ] BLE smoke path passes at least once per release build: connect -> write parameters -> `START` -> data notify -> `STOP`.
- [ ] Battery reporting and device-name write/read behavior verified on hardware.
- [ ] Sleep/idle spot-check confirms no obvious high-power regression.

## Versioning and Notes

- [ ] Update firmware version in `src/HWConfig/config.h`.
- [ ] Update `REVISION_HISTORY.md` with release title/date and key changes.
- [ ] Verify `README.md` reflects current technique support and file layout.
- [ ] Verify version string is consistent across `README.md`, `REVISION_HISTORY.md`, and the intended git tag (for example, `v1.1.0`).

## Build and Validation

- [ ] Build firmware (`pio run`).
- [ ] Flash and smoke test BLE connect/start/stop/data path.
- [ ] Verify battery reporting and device-name update behavior.
- [ ] Confirm no unintended high-power regressions.

## Repository Hygiene

- [ ] Ensure no temporary logs/artifacts are tracked.
- [ ] Confirm renamed/moved files are reflected in docs.
- [ ] Review release delta for unexpected files: `git diff --stat <last-tag>..HEAD`.
- [ ] Review full release commit list: `git log --oneline <last-tag>..HEAD`.
- [ ] Tag release commit after validation.
